#include <jni.h>
#include <string>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
using namespace std;
using namespace cv;

//extern "C" JNIEXPORT jstring JNICALL
//Java_io_github_evanchien_camera_1analysis2_MainActivity_stringFromJNI(
//        JNIEnv *env,
//        jobject /* this */) {
//
//
//
////    Mat& im = *(static_cast<Mat*>(addrImg));
//    std::string hello = "Hello from C++";
//    return env->NewStringUTF(hello.c_str());
//    cv::Mat temp_image,image, image_pad,image_pad_wb, image_hough,image_pad_hough, image_pad_gray, image_hsv, image_raw_hsv, image_manmade, image_manmade_hsv, image_checker;
//}
//extern "C"
//JNIEXPORT void JNICALL Java_com_panjq_opencv_alg_ImagePro_jniImagePro3
//        (JNIEnv *, jobject, jlong matAddrSrcImage, jlong matAddrDestImage){
//    Mat& srcImage  = *(Mat*)matAddrSrcImage;
//    Mat& destImage = *(Mat*)matAddrDestImage;
//    cv::cvtColor(srcImage,srcImage,CV_BGRA2BGR);
//    blur(srcImage,destImage,Size(20,20));
//    cv::cvtColor(destImage,destImage,CV_BGR2BGRA);
//}


int thresh = 200;
int max_thresh = 255;
RNG rng(12345);
int AREA_PER_ROW =6;
float hsv_delta = 10;
int SIZE = 4;
int height, width;


Mat temp_image,image, image_raw, image_pad,image_pad_wb, image_hough,image_pad_hough, image_pad_gray, image_hsv, image_raw_hsv, image_manmade, image_manmade_hsv, image_checker;
Mat image_pad_hsv;

tuple<vector<Point>, vector<Point>> area_grouping(vector<Point> area_list);
tuple<vector<Point>, vector<Point>> area_finder(vector<Point> group_1, vector<Point> group_2, cv::Mat& img_hsv, Mat& img_pad_hsv);
double pt_distance(vector<cv::Point> group);
double round(double r);
int area_expansion(vector<Point> group, vector<Point>& aug_group, Mat& img_pad_hsv, double min_distance);
void SimplestCB(cv::Mat& in, cv::Mat& checker, cv::Mat& out, float percent);

struct ptclass {
    bool operator() (cv::Point pt1, cv::Point pt2) { return (pt1.x < pt2.x);}
} ptobject;



extern "C"
JNIEXPORT jstring JNICALL
Java_com_example_evan_camera_1analysis2_MainActivity_matProcessing(JNIEnv *env, jobject instance,
                                                                   long matAddress, bool switchKey) {


//    int height, width;
    char* den;
    vector<cv::Point> group_1, group_2;
    Mat* mRgb = (Mat*)matAddress;
//    transpose(*mRgb, *mRgb);
//    flip(*mRgb, *mRgb,1);
    height = mRgb->rows;
    width = mRgb->cols;
    cv::Rect pad_area;


//    Defining pad position

    // pad area for picture
    pad_area.x = (int)(width/8);
    pad_area.y = (int)(height/8);
    pad_area.width = (int)(3*width/4);
    pad_area.height = (int)(3*height/4);

    // pad area for real sample
//    pad_area.x = (int)(3*width/8);
//    pad_area.y = (int)(3*height/8);
//    pad_area.width = (int)(width/8);
//    pad_area.height = (int)(height/8);



    cv::cvtColor(*mRgb, image_raw, cv::COLOR_BGRA2BGR);
//    cv::cvtColor(*mRgb,*mRgb, cv::COLOR_BGRA2GRAY);
//    cv::Canny(*mRgb, *mRgb,30,60);





    SimplestCB(image_raw, image_raw, image_hough, 0.1);
    cv::cvtColor(image_raw, image_raw_hsv, cv::COLOR_BGR2HSV);
//    image_pad = image_raw(pad_area);
    image_pad = image_hough(pad_area);
    image_pad_hough = image_hough(pad_area);
//    image_pad_hough = image_pad;
    cv::cvtColor(image_pad, image_pad_hsv, cv::COLOR_BGR2HSV);
    cv::cvtColor(image_pad_hough, image_pad_gray, COLOR_BGR2GRAY);
    vector<Vec3f> circles;
    vector<cv::Point> area_list;

    if (switchKey){
        cv::GaussianBlur( image_pad_gray, image_pad_gray, Size(7, 7), 1, 1 );
//        cv::HoughCircles( image_pad_gray, circles, HOUGH_GRADIENT, 1, 30, 20, 50, 30, 100 );
        cv::HoughCircles( image_pad_gray, circles, HOUGH_GRADIENT, 1, 100, 40, 50, 60, 100 );
        //HOUGH_GRADIENT, dp, minDist, CannyHighThreshold, counter thresh, minradius, maxradius
    }
    else{
        cv::GaussianBlur( image_pad_gray, image_pad_gray, Size(7, 7), 1, 1 );
        cv::HoughCircles( image_pad_gray, circles, HOUGH_GRADIENT, 1, 30, 20, 60, 30, 100 );

    }




//    cv::GaussianBlur( image_pad_gray, image_pad_gray, Size(7, 7), 1, 1 );
//    cv::GaussianBlur( image_pad_gray, image_pad_gray, Size(20, 20), 1, 1 );


    // HoughCircle for picture
//    cv::HoughCircles( image_pad_gray, circles, HOUGH_GRADIENT, 1, (int)(width/10), 80, 40, 40, 120 );

    // HoughCircle for sample
//    cv::HoughCircles( image_pad_gray, circles, HOUGH_GRADIENT, 1, 50, 80, 40, 40, 120 );
//    cv::HoughCircles( image_pad_gray, circles, HOUGH_GRADIENT, 1, 50, 80, 40, 20, 100 );




//    std::string hello = "# of object recognized: " + to_string(circles.size());
    std::string circle_cnt;

    if (circles.size() == 0)
    {
        circle_cnt = "No test area recognized";
        return env->NewStringUTF(circle_cnt.c_str());
    }
//    else if (circles.size() >6)
//    {
//        circle_cnt = "False detection recognized";
//        return env->NewStringUTF(circle_cnt.c_str());
//    }
    else{
        int cnt =0;
        for( size_t i = 0; i < circles.size(); i++ )
        {

            cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            // circle center

            // Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            // cout<<center.x<<", "<<center.y<<endl;
            // circle( image_pad_wb, center, 5, Scalar(0,255,0), -1, 8, 0 );

            // circle outline
            if(radius >60){
                circle( image_pad, center, radius, Scalar(0,0,255), 3, 8, 0 );
                cnt ++;
                area_list.push_back(center);
            }

        }
        tie(group_1, group_2) = area_grouping(area_list);
//        if (group_1.size()==0){
//            std::string gp1 = "Do not have enough points for augmentation in group 1"<<endl;
//            return -1;
//        }
//        else if (group_2.size()==0){
//            cout<<"Do not have enough points for augmentation in group 2"<<endl;
//            return -1;
//        }
//        else if (group_1.size()==1 && group_2.size() ==1)
//        {
//            cout<<"Do not have enough points for augmentation in group 1"<<endl;
//            return -1;
//        }
//        cout<<"group 1 before augmentation"<<group_1<<endl;
//        cout<<"group 2 before augmentation"<<group_2<<endl;

        tie(group_1, group_2) = area_finder(group_1, group_2, image_raw_hsv, image_pad_hsv);


    }
    Point3f raw_sum(0.0f, 0.0f, 0.0f);
    image_pad = image_raw(pad_area);
    for (int i=0;i<group_1.size();i++)
    {
        Scalar avg_1;
        Rect sample_area;
        Mat image_sample;
        sample_area.x = int(group_1[i].x-SIZE/2);
        sample_area.y = int(group_1[i].y-SIZE/2);
        sample_area.width = SIZE;
        sample_area.height = SIZE;
        image_sample = image_pad_hsv(sample_area);


        avg_1 = mean(image_sample);

        raw_sum.x += image_pad_hsv.at<Vec3b>(group_1[i])[0];
        raw_sum.y += image_pad_hsv.at<Vec3b>(group_1[i])[1];
        raw_sum.z += image_pad_hsv.at<Vec3b>(group_1[i])[2];
//        mycsv<<to_string(avg[0])<<";"<<to_string(avg[1])<<";"<<to_string(avg[2])<<endl;
    }
    for (int i=0;i<group_2.size();i++)
    {
        Scalar avg_2;
        Rect sample_area;
        Mat image_sample;
        sample_area.x = int(group_1[i].x-SIZE/2);
        sample_area.y = int(group_1[i].y-SIZE/2);
        sample_area.width = SIZE;
        sample_area.height = SIZE;
        image_sample = image_pad_hsv(sample_area);

        avg_2 = mean(image_sample);

        raw_sum.x += image_pad_hsv.at<Vec3b>(group_2[i])[0];
        raw_sum.y += image_pad_hsv.at<Vec3b>(group_2[i])[1];
        raw_sum.z += image_pad_hsv.at<Vec3b>(group_2[i])[2];
//        mycsv<<to_string(avg[0])<<";"<<to_string(avg[1])<<";"<<to_string(avg[2])<<endl;
    }
    Point3f raw_avg(raw_sum.x/(group_1.size()+group_2.size()),raw_sum.y/(group_1.size()+group_2.size()),raw_sum.z/(group_1.size()+group_2.size()));
//    std:string hsv = "Hue: " + std::to_string(raw_avg.x) +", Saturation: " + std::to_string(raw_avg.y) + ", Value: " + \
//            std::to_string(raw_avg.z);
//    cout<< "Fetching testing area"<<endl;
    std::string hsv = std::to_string((int)(raw_avg.y));

    std::string hello = "width: "+std::to_string(width)+" height: "+std::to_string(height);


//    cvtColor(*mRgb, img_gray,CV_BGRA2GRAY);

//    *mRgb = image_raw;
        *mRgb = image_hough;
//    return env->NewStringUTF(hello.c_str());
    return env->NewStringUTF(hsv.c_str());
}

double pt_distance(vector<Point> group){
    /*************************************************
      Helper function for distance between two points
    *************************************************/
    float distance =1000, temp_distance;
    for (size_t i =0; i <group.size()-1;i++)
    {
        for (size_t j =i+1; j<group.size();j++)
        {
            temp_distance = sqrt((group[i].x-group[j].x)*(group[i].x-group[j].x) + (group[i].y-group[j].y)*(group[i].y-group[j].y));
            if (temp_distance < distance)
            {
                distance = temp_distance;
            }
        }
    }
    return distance;
}

double round(double r)  {
    return (r > 0.0) ? floor(r + 0.5) : ceil(r - 0.5);
}

int area_expansion(vector<Point> group, vector<Point>& aug_group, Mat& image_pad_hsv, double min_distance){
    /*************************************************************************
      The function uses fit line and color matching to augment the test areas
      Argv:
        group - a group of points in vector
        aug_group - another group of points
        image_pad_hsv - image of target area in hsv space
        min_distance - a double showing the finimum distance between points for interpolation.
      return:
        int - -1 for failure of area extraction
    *************************************************************************/
    vector<Point> temp_group = group;
    for (int i=0;i<aug_group.size();i++)
    {
        temp_group.push_back(aug_group[i]);
    }
    sort(temp_group.begin(), temp_group.end(), ptobject);
//    cout<<"temp_group = "<<temp_group<<endl;
    if (temp_group.size()<=1)
    {
        return -1;
    }
    else
    {
        Vec4f temp_line;
        cv::fitLine(temp_group, temp_line, DIST_L2, 0,1e-2, 1e-2);
        double a = temp_line[1]/temp_line[0];
        double b = temp_line[3]- a*temp_line[2];


        Point now_pt = temp_group[0];
        Point front_pt;
        Vec3f avg_hsv;
        Vec3f sum_hsv(0.00f, 0.00f, 0.00f);
        front_pt.x = temp_group[0].x-min_distance*cos(atan(a));
        front_pt.y = a*front_pt.x + b;
        Vec3b front_hsv = image_pad_hsv.at<Vec3b>(front_pt);

        for (int i =0;i<temp_group.size();i++)
        {
            sum_hsv[0]+= image_pad_hsv.at<Vec3b>(temp_group[i])[0];
            sum_hsv[1]+= image_pad_hsv.at<Vec3b>(temp_group[i])[1];
            sum_hsv[2]+= image_pad_hsv.at<Vec3b>(temp_group[i])[2];
        }
        avg_hsv[0] = sum_hsv[0]/temp_group.size();
        avg_hsv[1] = sum_hsv[1]/temp_group.size();
        avg_hsv[2] = sum_hsv[2]/temp_group.size();
//        cout<<"avg pt hsv = "<<avg_hsv<<endl;
        while (1)
        {
            if (temp_group.size()>=6)
            {
                break;
            }
            else{
                // Front expansion

//                cout<<"front pt at "<<front_pt<<" hsv = "<<front_hsv<<endl;;
                double temp_hsv_delta = abs(front_hsv[1]- avg_hsv[1]);
//                cout<<"hsv delta = "<<temp_hsv_delta<<endl;
                if (temp_hsv_delta <hsv_delta && front_pt.x >=0)
                {
                    aug_group.push_back(front_pt);
                    temp_group.push_back(front_pt);
                    now_pt = front_pt;
                    front_pt.x = now_pt.x-min_distance*cos(atan(a));
                    front_pt.y = a*now_pt.x + b;
                    temp_hsv_delta = abs(front_hsv[1]-avg_hsv[1]);
                }
                else{
                    break;
                }
            }
        }
        sort(temp_group.begin(), temp_group.end(), ptobject);
        now_pt = temp_group.back();
        Point back_pt;
        sum_hsv[0] = 0.00;
        sum_hsv[1] = 0.00;
        sum_hsv[2] = 0.00;
        back_pt.x = temp_group.back().x+min_distance*cos(atan(a));
        back_pt.y = a*back_pt.x + b;
        Vec3b back_hsv = image_pad_hsv.at<Vec3b>(back_pt);

        for (int i =0;i<temp_group.size();i++)
        {
            sum_hsv[0]+= image_pad_hsv.at<Vec3b>(temp_group[i])[0];
            sum_hsv[1]+= image_pad_hsv.at<Vec3b>(temp_group[i])[1];
            sum_hsv[2]+= image_pad_hsv.at<Vec3b>(temp_group[i])[2];
        }
        avg_hsv[0] = sum_hsv[0]/temp_group.size();
        avg_hsv[1] = sum_hsv[1]/temp_group.size();
        avg_hsv[2] = sum_hsv[2]/temp_group.size();
//        cout<<"avg pt hsv = "<<avg_hsv<<endl;

        while(1)
        {
            if (temp_group.size()>=6)
            {
                break;
            }
            else
            {
                // back expansion

//                cout<<"back pt at "<<back_pt<<" hsv = "<<back_hsv<<endl;;
                double temp_hsv_delta = abs(back_hsv[1]- avg_hsv[1]);
//                cout<<"hsv delta = "<<temp_hsv_delta<<endl;
                if (temp_hsv_delta <hsv_delta && back_pt.x <= ::width)
                {
                    aug_group.push_back(back_pt);
                    temp_group.push_back(back_pt);
                    now_pt = back_pt;
                    back_pt.x = now_pt.x+min_distance*cos(atan(a));
                    back_pt.y = a*back_pt.x + b;
                    temp_hsv_delta = abs(back_hsv[1]-avg_hsv[1]);
                }
                else{
                    break;
                }
            }
        }
    }



    for (int i = 0;i <aug_group.size();i++)
    {
        circle( image_pad, aug_group[i], 30, Scalar(0,255,0), 3, 8, 0 );
    }
    return 0;
}

tuple<vector<Point>, vector<Point>> area_grouping(vector<Point> area_list){
    /*
      Helper function to seperate the points into two rows and return the two groups of vector<cv::Point>
      argv:
        area_list - A vector of point candidates
      return:
        A tuple of point vectors
    */
    double temp_y = 0, dis_threshold = (int)(::height/6);
    vector<Point> group_1, group_2;
    temp_y = area_list[0].y;
    group_1.push_back(area_list[0]);
    for (int j =1; j<area_list.size();j++)
    {
        if (abs(area_list[j].y -temp_y) > dis_threshold)
        {
            group_2.push_back(area_list[j]);
        }
        else
        {
            group_1.push_back(area_list[j]);
        }
    }
    sort(group_1.begin(), group_1.end(), ptobject);
    sort(group_2.begin(), group_2.end(), ptobject);
    return make_tuple(group_1, group_2);
}

tuple<vector<Point>, vector<Point>> area_finder(vector<Point> group_1, vector<Point> group_2, cv::Mat& img_hsv, Mat& image_pad_hsv){
    /********************************************************************************************
      The group augmentation function.
      1. Find the minimum distance between points of each group
      2. Do the interpolation between the points.
      3. Pass the interpolated groups to area_expansion function for front/back point expansion

      argv :
        group_1 - 1st group of points in vector
        group_2 - 2nd group of points in vector
        img_hsv - the input image in hsv space
        img_pad_hsv - the target area in hsv space
      returns:
        A tuple of two vector of points
    **********************************************************************************************/
    Vec4f line_1, line_2;
    // cout<< area_list<<endl;
    double temp_y = 0;
    vector<Point> aug_group_1, aug_group_2;



    double min_distance_group1 =1000, min_distance_group2 = 1000, min_distance;

    if (group_1.size()>1)
    {
        min_distance_group1 = pt_distance(group_1);
    }

    if (group_2.size()>1)
    {
        min_distance_group2 = pt_distance(group_2);

    }
    min_distance = min(min_distance_group1, min_distance_group2);

    // fill the areas between captured areas of group_1 and group_2
    if (group_1.size()<6 && group_1.size()>1)
    {
        for (size_t i=0; i<group_1.size()-1;i++)
        {
            double dist;
            int number_fill;
            dist = sqrt((group_1[i].x - group_1[i+1].x)*(group_1[i].x - group_1[i+1].x)+(group_1[i].y - group_1[i+1].y)*(group_1[i].y - group_1[i+1].y));
            number_fill = int(round(dist/min_distance))- 1;
            // cout<<"Number_fill = "<<number_fill<<endl;
            if (number_fill >0)
            {
                for (size_t j=1;j<number_fill+1;j++)
                {
                    int pt_x, pt_y;
                    // cout<<group_1[i]<<", "<<group_1[i+1]<<endl;
                    pt_x = int(group_1[i].x + (group_1[i+1].x - group_1[i].x)*double(j)/(number_fill+1));
                    pt_y = int(group_1[i].y + (group_1[i+1].y - group_1[i].y)*double(j)/(number_fill+1));
                    aug_group_1.push_back(Point(pt_x, pt_y));
                }
            }
        }
    }
    if (group_2.size()<6 && group_2.size()>1)
    {
        for (size_t i=0; i<group_2.size()-1;i++)
        {
            double dist;
            int number_fill;
            dist = sqrt((group_2[i].x - group_2[i+1].x)*(group_2[i].x - group_2[i+1].x)+(group_2[i].y - group_2[i+1].y)*(group_2[i].y - group_2[i+1].y));
            number_fill = int(round(dist/min_distance))- 1;
            // cout<<"Number_fill = "<<number_fill<<endl;
            if (number_fill >0)
            {
                for (size_t j=1;j<number_fill+1;j++)
                {
                    int pt_x, pt_y;
                    // cout<<group_1[i]<<", "<<group_1[i+1]<<endl;
                    pt_x = int(group_2[i].x + (group_2[i+1].x - group_2[i].x)*double(j)/(number_fill+1));
                    pt_y = int(group_2[i].y + (group_2[i+1].y - group_2[i].y)*double(j)/(number_fill+1));
                    aug_group_2.push_back(Point(pt_x, pt_y));
                }
            }
        }
    }
    // cout<<"Finish interpolation"<<endl;

    int tmp1 = area_expansion(group_1, aug_group_1, image_pad_hsv, min_distance);
    int tmp2 = area_expansion(group_2, aug_group_2, image_pad_hsv, min_distance);
    if (tmp1 == -1 ||tmp2 == -1)
    {
//        cout<<"One or more of the groups has only one or less area detected"<<endl;
    }
    for (int i=0;i<aug_group_1.size();i++)
    {
        group_1.push_back(aug_group_1[i]);
    }
    sort(group_1.begin(), group_1.end(), ptobject);
    for (int i=0;i<aug_group_2.size();i++)
    {
        group_2.push_back(aug_group_2[i]);
    }
    sort(group_2.begin(), group_2.end(), ptobject);
    // cout<<"Group_1 after merging"<<group_1<<endl;
    // cout<<"Group_2 after merging"<<group_2<<endl;

    return make_tuple(group_1, group_2);
}


void SimplestCB(Mat& in, Mat& checker, Mat& out, float percent) {
    /*
    Helper function to do the white balance correction and the contrast enhancement
    Mat& in : The input Mat
    Mat& checker : The correction reference area
    percent : The cutoff percentage
    */

    assert(in.channels() == 3);
    assert(percent > 0 && percent < 100);

    float half_percent = percent / 200.0f;

    vector<Mat> tmpsplit, tmpsplit_checker;
    split(in,tmpsplit);
    split(checker, tmpsplit_checker);
    for(int i=0;i<3;i++) {
        //find the low and high precentile values (based on the input percentile)
        Mat flat;
        tmpsplit_checker[i].reshape(1,1).copyTo(flat);
        cv::sort(flat,flat,SORT_EVERY_ROW + SORT_ASCENDING);
        int lowval = flat.at<uchar>(cvFloor(((float)flat.cols) * half_percent));
        int highval = flat.at<uchar>(cvCeil(((float)flat.cols) * (1.0 - half_percent)));
        // cout << lowval << " " << highval << endl;

        //saturate below the low percentile and above the high percentile
        tmpsplit[i].setTo(lowval,tmpsplit[i] < lowval);
        tmpsplit[i].setTo(highval,tmpsplit[i] > highval);

        //scale the channel
        normalize(tmpsplit[i],tmpsplit[i],0,255,NORM_MINMAX);
    }
    merge(tmpsplit,out);
}
