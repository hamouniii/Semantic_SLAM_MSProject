#include "ShapeImageProcessor.h"
using namespace std;
using namespace cv;

#define IMSHOW_MODE 0

ShapeImageProcessor::ShapeImageProcessor(void)
{

}


ShapeImageProcessor::~ShapeImageProcessor(void)
{
}

bool ShapeImageProcessor::readConfigs(std::string configFile)
{
    try
    {
        XMLFileReader my_xml_reader(configFile);

        double item_radius_init = my_xml_reader.readDoubleValue("ShapeColor_ObjectDetector_config:ShapeImageProcessor:ItemParameters:radius");
        double item_height_init = my_xml_reader.readDoubleValue("ShapeColor_ObjectDetector_config:ShapeImageProcessor:ItemParameters:height");

        double bucket_radius_init = my_xml_reader.readDoubleValue("ShapeColor_ObjectDetector_config:ShapeImageProcessor:BucketParameters:radius");
        double bucket_height_init = my_xml_reader.readDoubleValue("ShapeColor_ObjectDetector_config:ShapeImageProcessor:BucketParameters:height");

        item_width = item_radius_init * 2;
        item_height = item_height_init;

        bucket_width = bucket_radius_init * 2;
        bucket_height = bucket_height_init;

        cout<<"Item Parameters:"<<endl<<"Item Width: "<<item_width<<" ; "<<"Item Height: "<<item_height<<endl;
        cout<<"Bucket Parameters:"<<endl<<"Bucket Width: "<<bucket_width<<" ; "<<"Bucket Height: "<<bucket_height<<endl;
    }
    catch ( cvg_XMLFileReader_exception &e)
    {
        throw cvg_XMLFileReader_exception(std::string("[cvg_XMLFileReader_exception! caller_function: ") + BOOST_CURRENT_FUNCTION + e.what() + "]\n");
    }
}

void ShapeImageProcessor::setEllipseDetectorParameter(Mat &I)
{
    // Parameters Settings (Sect. 4.2)
    Size sz = I.size();
    int		iThLength = 8;
    float	fThObb = 3.0f;
    float	fThPos = 1.0f;
    float	fTaoCenters = 0.01f;
    int 	iNs = 16;
    float	fMaxCenterDistance = sqrt(float(sz.width*sz.width + sz.height*sz.height)) * fTaoCenters;
    float	fThScoreScore = 0.8f;

    // Gaussian filter parameters, in pre-processing
    Size	szPreProcessingGaussKernelSize = Size(5, 5);
    double	dPreProcessingGaussSigma = 1.0;

    float	fDistanceToEllipseContour = 0.1f;	// (Sect. 3.3.1 - Validation)
    float	fMinReliability = 0.4f;	// Const parameters to discard bad ellipses


    // Initialize Ellipse Detector with selected parameters
    ellipseDetector.SetParameters(szPreProcessingGaussKernelSize,
        dPreProcessingGaussSigma,
        fThPos,
        fMaxCenterDistance,
        iThLength,
        fThObb,
        fDistanceToEllipseContour,
        fThScoreScore,
        fMinReliability,
        iNs
        );
}

std::vector<cv::Point3f> ShapeImageProcessor::get3DObjectCorners(std::string object_name)
{
    std::vector<cv::Point3f> points_3D;
    if(object_name == "Item")
    {
        //  xyz opencv
//        points_3D.push_back(cv::Point3f(0.0, 0.0, 0.0));
//        points_3D.push_back(cv::Point3f(item_width, 0.0, 0.0));
//        points_3D.push_back(cv::Point3f(item_width, item_height, 0.0));
//        points_3D.push_back(cv::Point3f(0.0, item_height, 0.0));

        //  xyz Object coordinates (3D center of object)
        points_3D.push_back(cv::Point3f(-item_width/2.0, -item_height/2.0, item_width/2.0));
        points_3D.push_back(cv::Point3f(item_width/2.0, -item_height/2.0, item_width/2.0));
        points_3D.push_back(cv::Point3f(item_width/2.0, item_height/2.0, -item_width/2.0));
        points_3D.push_back(cv::Point3f(-item_width/2.0, item_height/2.0, -item_width/2.0));
    }
    if(object_name == "Bucket")
    {
        //  xyz Object coordinates (3D center of object)
        points_3D.push_back(cv::Point3f(-bucket_width/2.0, -bucket_height/2.0, bucket_width/2.0));
        points_3D.push_back(cv::Point3f(bucket_width/2.0, -bucket_height/2.0, bucket_width/2.0));
        points_3D.push_back(cv::Point3f(bucket_width/2.0, bucket_height/2.0, -bucket_width/2.0));
        points_3D.push_back(cv::Point3f(-bucket_width/2.0, bucket_height/2.0, -bucket_width/2.0));

        //Circunference of the top of the Bucket
//        points_3D.push_back(cv::Point3f(-bucket_width/2.0, -bucket_height/2.0, bucket_width/2.0));
//        points_3D.push_back(cv::Point3f(bucket_width/2.0, -bucket_height/2.0, bucket_width/2.0));
//        points_3D.push_back(cv::Point3f(bucket_width/2.0, -bucket_height/2.0, -bucket_width/2.0));
//        points_3D.push_back(cv::Point3f(-bucket_width/2.0, -bucket_height/2.0, -bucket_width/2.0));
    }

    return points_3D;
}

cv::Mat ShapeImageProcessor::detectEdges(Mat &I)
{
    //For posterior 3D object Detection based on EDGES
    cv::Mat I_canny, I_gray;
    I.copyTo(I_canny);
    cv::cvtColor(I, I_gray, CV_BGR2GRAY);

    cv::Canny(I_gray, I_canny, 20, 100);
    cv::dilate(I_canny, I_canny, cv::Mat(), cv::Point(-1,-1));

    return I_canny;
}

std::vector<double> ShapeImageProcessor::computeHuMoments(std::vector<Point> &contour)
{
    std::vector<double> hu_moments;
    cv::Moments mom = cv::moments(contour);
    double hu[7];
    cv::HuMoments(mom, hu);

    for(unsigned int i=0;i<7;i++)
        hu_moments.push_back(hu[i]);

    return hu_moments;
}

cv::Mat ShapeImageProcessor::detectRectangles(Mat &I)
{
    vector<vector<Point> > rectangles;

    findRectangles(I, rectangles);
    cv::Mat I_mask = drawRectanglesMask(I, rectangles);
    //cv::Mat I_mask = drawRectanglesContour(I, rectangles);

    return I_mask;
}

void ShapeImageProcessor::findRectangles(const cv::Mat& image, std::vector<std::vector<cv::Point> >& squares)
{
   squares.clear();

    Mat pyr, timg, gray0(image.size(), CV_8U), gray;

    // down-scale and upscale the image to filter out the noise
    pyrDown(image, pyr, Size(image.cols/2, image.rows/2));
    pyrUp(pyr, timg, image.size());
    vector<vector<Point> > contours;

    // find squares in every color plane of the image
    for( int c = 0; c < 3; c++ )
    {
        int ch[] = {c, 0};
        mixChannels(&timg, 1, &gray0, 1, ch, 1);

        cv::Canny(gray0, gray, 50, 100);
        dilate(gray, gray, Mat(), Point(-1,-1));
        imshow("Canny", gray);

        // find contours and store them all as a list
        findContours(gray, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

        vector<Point> approx;
        // test each contour
        for( size_t i = 0; i < contours.size(); i++ )
        {
            // approximate contour with accuracy proportional
            // to the contour perimeter
            approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);

            // square contours should have 4 vertices after approximation
            // relatively large area (to filter out noisy contours)
            // and be convex.
            // Note: absolute value of an area is used because
            // area may be positive or negative - in accordance with the
            // contour orientation
            if( approx.size() == 4 && fabs(contourArea(Mat(approx))) > 1000 && isContourConvex(Mat(approx)) )
            {
                double maxCosine = 0;

                for( int j = 2; j < 5; j++ )
                {
                    // find the maximum cosine of the angle between joint edges
                    double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                    maxCosine = MAX(maxCosine, cosine);
                }

                // if cosines of all angles are small
                // (all angles are ~90 degree) then write quandrange
                // vertices to resultant sequence
                if( maxCosine < 0.3 )
                    squares.push_back(approx);
            }

        }
    }


}

Mat ShapeImageProcessor::drawRectanglesMask(cv::Mat& image, const std::vector<std::vector<cv::Point> >& squares)
{
    cv::Mat I_mask = cv::Mat::zeros(image.size(), CV_8U);
    //cout<<"Drawing rectangles masks"<<endl;
    for( size_t i = 0; i < squares.size(); i++ )
    {
        const Point* p = &squares[i][0];
        int n = (int)squares[i].size();
        cv::fillConvexPoly(I_mask, p, n, Scalar(255));
    }

    imshow("Squares mask", I_mask);
    return I_mask;
}

cv::Mat ShapeImageProcessor::drawRectanglesContour(Mat& image, const vector<vector<Point> >& squares)
{
    cv::Mat I_mask = cv::Mat::zeros(image.size(), CV_8U);
    for( size_t i = 0; i < squares.size(); i++ )
    {
        const Point* p = &squares[i][0];
        int n = (int)squares[i].size();
        cv::polylines(I_mask, &p, &n, 1, true, Scalar(255), 3);
    }

    imshow("Squares contours", I_mask);
    return I_mask;
}

double ShapeImageProcessor::angle(cv::Point pt1, cv::Point pt2, cv::Point pt0)
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

cv::RotatedRect ShapeImageProcessor::detectEllipseInObject(const Mat &I, cv::RotatedRect rot_rect)
{
    cv::Mat I_gray, I_best_ellipse_in_image;
    cv::cvtColor(I, I_gray, CV_BGR2GRAY);
    I.copyTo(I_best_ellipse_in_image);


    vector<Ellipse> ellipses_red, ellipses_blue;
    vector<Ellipse> ellipses;
    vector<Ellipse> ellipses_inside_rect;
    vector<Ellipse> best_ellipse_inside_rect;
    cv::Mat1b I_gray_crop_1b;
    cv::Mat3b I_ellipses_black = cv::Mat3b::zeros(I.rows, I.cols);
    cv::Mat3b I_ellipses_inside_rect_black = cv::Mat3b::zeros(I.rows, I.cols);
    cv::Mat1b I_best_ellipse_inside_rect_black;
    cv::Rect object_rect = rot_rect.boundingRect();


    std::vector<cv::Mat> canales_rgb;
    cv::split(I, canales_rgb);
    cv::Mat1b red_ch = canales_rgb[2];
    cv::Mat1b blue_ch = canales_rgb[0];
    I_gray_crop_1b = I_gray;
    ellipseDetector.Detect(red_ch, ellipses_red);
    //ellipseDetector.Detect(blue_ch, ellipses_blue);
    ellipses.insert(ellipses.end(), ellipses_red.begin(), ellipses_red.end());
    //ellipses.insert(ellipses.end(), ellipses_blue.begin(), ellipses_blue.end());
    ellipseDetector.DrawDetectedEllipses(I_ellipses_black, ellipses);



    for(unsigned int i=0;i<ellipses.size();i++)
    {
        if(((ellipses[i]._xc > object_rect.x) && (ellipses[i]._xc < (object_rect.x + object_rect.width))) &&
                ((ellipses[i]._yc > object_rect.y) && (ellipses[i]._yc < (object_rect.y + object_rect.height))))
        {
            ellipses_inside_rect.push_back(ellipses[i]);
        }
    }


    std::vector<float> ellipse_scores;
    for(unsigned int i=0;i<ellipses_inside_rect.size();i++)
        ellipse_scores.push_back(ellipses_inside_rect[i]._score);

    double max_score = 0.0;
    if(ellipse_scores.size())
        max_score = *std::max_element(ellipse_scores.begin(), ellipse_scores.end());
    //cout<<"Score of Best Ellipse: "<<max_score<<endl;


    for(unsigned int i=0;i<ellipses_inside_rect.size();i++)
    {
        if(ellipses_inside_rect[i]._score == max_score)
            best_ellipse_inside_rect.push_back(ellipses_inside_rect[i]);
    }
    ellipseDetector.DrawDetectedEllipses(I_ellipses_inside_rect_black, best_ellipse_inside_rect);



    cv::inRange(I_ellipses_inside_rect_black, cv::Scalar(0, 1, 0), cv::Scalar(255, 255, 255), I_best_ellipse_inside_rect_black);

    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(I_best_ellipse_inside_rect_black, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    cv::RotatedRect boxBestEllipse;
    if(contours.size())
    {
        boxBestEllipse = minAreaRect(cv::Mat(contours[0]));
        Point2f rect_points[4];
        boxBestEllipse.points(rect_points);
        for( int j = 0; j < 4; j++ )
        {
            cv::line(I_best_ellipse_in_image, rect_points[j], rect_points[(j+1)%4], cv::Scalar(0, 255, 0), 2, 8);
        }
    }

    if(IMSHOW_MODE)
    {
        cv::imshow("Detected Ellipses", I_ellipses_black);
        cv::imshow("BEST Ellipse Inside Object Rect", I_ellipses_inside_rect_black);
        cv::imshow("Best Ellipse in Image", I_best_ellipse_in_image);
        cv::imshow("Red Channel", red_ch);
        cv::imshow("Blue Channel", blue_ch);
        cv::waitKey(1);
    }

    return boxBestEllipse;

}




