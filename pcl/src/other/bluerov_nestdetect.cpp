#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <iostream>

using namespace std;
using namespace cv;

static void help()
{
    cout << "\nThis program demonstrates the use of cv::CascadeClassifier class to detect objects (Face + eyes). You can use Haar or LBP features.\n"
            "This classifier can recognize many kinds of rigid objects, once the appropriate classifier is trained.\n"
            "It's most known use is for faces.\n"
            "Usage:\n"
            "./facedetect [--cascade=<cascade_path> this is the primary trained classifier such as frontal face]\n"
               "   [--nested-cascade[=nested_cascade_path this an optional secondary classifier such as eyes]]\n"
               "   [--scale=<image scale greater or equal to 1, try 1.3 for example>]\n"
               "   [--try-flip]\n"
               "   [filename|camera_index]\n\n"
            "see facedetect.cmd for one call:\n"
            "./facedetect --cascade=\"data/haarcascades/haarcascade_frontalface_alt.xml\" --nested-cascade=\"data/haarcascades/haarcascade_eye_tree_eyeglasses.xml\" --scale=1.3\n\n"
            "During execution:\n\tHit any key to quit.\n"
            "\tUsing OpenCV version " << CV_VERSION << "\n" << endl;
}

void detectAndDraw( Mat& img, CascadeClassifier& cascade,
                    CascadeClassifier& nestedCascade,
                    double scale, bool tryflip );

string cascadeName;
string nestedCascadeName;

int main( int argc, const char** argv )
{
    VideoCapture capture;
    Mat frame, image;
    string inputName;
    bool tryflip;
    CascadeClassifier cascade, nestedCascade;
    double scale;

    cv::CommandLineParser parser(argc, argv,
        "{help h||}"
        "{cascade|data/haarcascades/haarcascade_frontalface_alt.xml|}"
        "{nested-cascade|data/haarcascades/haarcascade_eye_tree_eyeglasses.xml|}"
        "{scale|1|}{try-flip||}{@filename||}"
    );
    if (parser.has("help"))
    {
        help();
        return 0;
    }
    cascadeName = parser.get<string>("cascade");
    nestedCascadeName = parser.get<string>("nested-cascade");
    scale = parser.get<double>("scale");
    if (scale < 1)
        scale = 1;
    tryflip = parser.has("try-flip");
    inputName = parser.get<string>("@filename");
    if (!parser.check())
    {
        parser.printErrors();
        return 0;
    }

    if (!nestedCascade.load(samples::findFileOrKeep(nestedCascadeName)))
        cerr << "WARNING: Could not load classifier cascade for nested objects" << endl;
    if (!cascade.load(samples::findFile(cascadeName)))
    {
        cerr << "ERROR: Could not load classifier cascade" << endl;
        help();
        return -1;
    }
    
    if( inputName.empty() || (isdigit(inputName[0]) && inputName.size() == 1) )
    {
        int camera = inputName.empty() ? 0 : inputName[0] - '0';
        if(!capture.open(camera))
        {
            cout << "Capture from camera #" <<  camera << " didn't work" << endl;
            return 1;
        }
    }
    else if (!inputName.empty())
    {
        image = imread(samples::findFileOrKeep(inputName), IMREAD_COLOR);
        if (image.empty())
        {
            if (!capture.open(samples::findFileOrKeep(inputName)))
            {
                cout << "Could not read " << inputName << endl;
                return 1;
            }
        }
    }
    else
    {
        image = imread(samples::findFile("lena.jpg"), IMREAD_COLOR);
        if (image.empty())
        {
            cout << "Couldn't read lena.jpg" << endl;
            return 1;
        }
    }

    if( capture.isOpened() )
    {
        cout << "Video capturing has been started ..." << endl;

        for(;;)
        {
            capture >> frame;
            if( frame.empty() )
                break;

            Mat frame1 = frame.clone();
            detectAndDraw( frame1, cascade, nestedCascade, scale, tryflip );

            char c = (char)waitKey(10);
            if( c == 27 || c == 'q' || c == 'Q' )
                break;
        }
    }
    else
    {
        cout << "Detecting face(s) in " << inputName << endl;
        if( !image.empty() )
        {
            detectAndDraw( image, cascade, nestedCascade, scale, tryflip );
            waitKey(0);
        }
        else if( !inputName.empty() )
        {
            /* assume it is a text file containing the
            list of the image filenames to be processed - one per line */
            FILE* f = fopen( inputName.c_str(), "rt" );
            if( f )
            {
                char buf[1000+1];
                while( fgets( buf, 1000, f ) )
                {
                    int len = (int)strlen(buf);
                    while( len > 0 && isspace(buf[len-1]) )
                        len--;
                    buf[len] = '\0';
                    cout << "file " << buf << endl;
                    image = imread( buf, 1 );
                    if( !image.empty() )
                    {
                        detectAndDraw( image, cascade, nestedCascade, scale, tryflip );
                        char c = (char)waitKey(0);
                        if( c == 27 || c == 'q' || c == 'Q' )
                            break;
                    }
                    else
                    {
                        cerr << "Aw snap, couldn't read image " << buf << endl;
                    }
                }
                fclose(f);
            }
        }
    }

    return 0;
}

void detectAndDraw( Mat& img, CascadeClassifier& cascade,
                    CascadeClassifier& nestedCascade,
                    double scale, bool tryflip )
{
    double t = 0;
    vector<Rect> faces, faces2;
    Mat gray, smallImg;

    Size minSize = Size(smallImg.cols*0.1, smallImg.rows*0.1);
    Size maxSize = Size(smallImg.cols*0.3, smallImg.rows*0.3);   
    int nearNum1 = 50, nearNum2 = 10; 

    cvtColor( img, gray, COLOR_BGR2GRAY );
    double fx = 1 / scale;
    resize( gray, smallImg, Size(), fx, fx, INTER_LINEAR_EXACT );
    equalizeHist( smallImg, smallImg );

    cout<<"img  w,h: "<<img.cols<<" "<<img.rows<<endl;
    cout<<"smallimg  w,h: "<<smallImg.cols<<" "<<smallImg.rows<<endl;

    t = (double)getTickCount();
    cascade.detectMultiScale( smallImg, faces,
        1.1, nearNum1, 0
        |CASCADE_FIND_BIGGEST_OBJECT,
        //|CASCADE_DO_ROUGH_SEARCH
        // |CASCADE_SCALE_IMAGE,
        minSize, maxSize);
    // if( tryflip )
    // {
    //     flip(smallImg, smallImg, 1);
    //     cascade.detectMultiScale( smallImg, faces2,
    //                              1.1, nearNum1, 0
    //                              //|CASCADE_FIND_BIGGEST_OBJECT
    //                              //|CASCADE_DO_ROUGH_SEARCH
    //                              |CASCADE_SCALE_IMAGE,
    //                              Size(30, 30) );
    //     for( vector<Rect>::const_iterator r = faces2.begin(); r != faces2.end(); ++r )
    //     {
    //         faces.push_back(Rect(smallImg.cols - r->x - r->width, r->y, r->width, r->height));
    //     }
    // }
    t = (double)getTickCount() - t;
    printf( "1st round detection time = %g ms\n", t*1000/getTickFrequency());


    for ( size_t i = 0; i < faces.size(); i++ )
    {
        Rect r = faces[i];
        cout<<"r origin:  "<<r.x<<"  "<<r.y<<endl;
        cout<<"r w h:  "<<r.width<<"  "<<r.height<<endl;
        Rect r2;
        r2 = r;
        cout<<"r2 origin:  "<<r2.x<<"  "<<r2.y<<endl;
        cout<<"r2 w h:  "<<r2.width<<"  "<<r2.height<<endl;

        float rScale = 1.2;
        r2.width = r.width*rScale;
        r2.height = r.height*1; 
        float rCenter = r.x + r.width/2;
        // cout<<"r center = "<<rCenter<<endl;
        // cout<<"r2 width/2 = "<<r2.width*0.5<<endl;        
        r2.x = rCenter - r2.width/2;      
        cout<<"r22  origin: "<<r2.x<<"  "<<r2.y<<endl;
        cout<<"r22 w h:  "<<r2.width<<"  "<<r2.height<<endl;

        Mat smallImgROI;
        vector<Rect> nestedObjects;
        Scalar color1 = Scalar(0,255,0); //BLUE

        // rectangle( img, Point(cvRound(r.x*scale), cvRound(r.y*scale)),
        //             Point(cvRound((r.x + r.width-1)*scale), cvRound((r.y + r.height-1)*scale)),
        //             color1, 3, 8, 0);

        rectangle( img, Point(cvRound(r2.x*scale), cvRound(r2.y*scale)),
                    Point(cvRound((r2.x + r2.width-1)*scale), cvRound((r2.y + r2.height-1)*scale)),
                    color1, 3, 8, 0);                    
        cout<<endl;

        if( nestedCascade.empty() )
            continue;

        smallImgROI = smallImg( r2 );
        cout<<"smallImgROI w,h: "<<smallImgROI.cols<<" "<<smallImgROI.rows<<endl;
        Size minSize2 = Size(smallImgROI.cols*0.6, smallImgROI.rows*0.2);
        Size maxSize2 = Size(smallImgROI.cols*1, smallImgROI.rows*0.5);  

        t = (double)getTickCount();
        nestedCascade.detectMultiScale( smallImgROI, nestedObjects,
            1.1, nearNum2, 0
            |CASCADE_FIND_BIGGEST_OBJECT,
            //|CASCADE_DO_ROUGH_SEARCH
            //|CASCADE_DO_CANNY_PRUNING
            // |CASCADE_SCALE_IMAGE,
            minSize2, maxSize2 );
        t = (double)getTickCount() - t;
        printf( "2nd round detection time = %g ms\n", t*1000/getTickFrequency());

        for ( size_t j = 0; j < nestedObjects.size(); j++ )
        {
            Rect nr = nestedObjects[j];
            Scalar color2 = Scalar(255,0,0);  //BGR     
            Scalar color12 = Scalar(0,0,255);  //BGR     

            // nested object
            // rectangle( img, Point(cvRound((r.x+nr.x)*scale), cvRound((r.y+nr.y)*scale)),
            //            Point(cvRound((r.x + nr.width-1)*scale), cvRound((r.y + nr.height-1)*scale)),
            //            color2, 3, 8, 0);
            rectangle( img, Point(cvRound((r2.x+nr.x)*scale), cvRound((r2.y+nr.y)*scale)),
                       Point(cvRound((r2.x + nr.width-1)*scale), cvRound((r2.y + nr.height-1)*scale)),
                       color2, 3, 8, 0);            

            // confirmed object
            rectangle( img, Point(cvRound(r.x*scale), cvRound(r.y*scale)),
                        Point(cvRound((r.x + r.width-1)*scale), cvRound((r.y + r.height-1)*scale)),
                        color12, 3, 8, 0);                       
        }
        cout<<"---------------------------"<<endl;
    }
    cout<<"=================================="<<endl;

    imshow( "result", img );
}
