#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

using namespace std;

void detKeypoints1()
{
    // load image from file and convert to grayscale
    cv::Mat imgGray;
    cv::Mat img = cv::imread("../images/img1.png");
    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

    // Shi-Tomasi detector
    int blockSize = 6;       //  size of a block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints
    double qualityLevel = 0.01;                                   // minimal accepted quality of image corners
    double k = 0.04;
    bool useHarris = false;

    vector<cv::KeyPoint> kptsShiTomasi;
    vector<cv::Point2f> corners;
    double t = (double)cv::getTickCount();
    cv::goodFeaturesToTrack(imgGray, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, useHarris, k);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Shi-Tomasi with n= " << corners.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    for (auto it = corners.begin(); it != corners.end(); ++it)
    { // add corners to result vector

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        kptsShiTomasi.push_back(newKeyPoint);
    }

    // visualize results
    cv::Mat visImage = img.clone();
    cv::drawKeypoints(img, kptsShiTomasi, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    string windowName = "Shi-Tomasi Results";
    cv::namedWindow(windowName, 1);
    imshow(windowName, visImage);

    // TODO: use the OpenCV library to add the FAST detector
    // in addition to the already implemented Shi-Tomasi 
    // detector and compare both algorithms with regard to 
    // (a) number of keypoints, (b) distribution of 
    // keypoints over the image and (c) processing speed.

    // FAST detector
    int threshold = 30; // difference between intensity of the central pixel and pixels of a circle around this pixel
    bool bNMS = true;   // perform non-maxima suppression on keypoints
    cv::FastFeatureDetector::DetectorType type = cv::FastFeatureDetector::TYPE_9_16;
    cv::Ptr<cv::FeatureDetector> detector = cv::FastFeatureDetector::create(threshold, bNMS, type);
    vector<cv::KeyPoint> kptsFAST;
    t = (double)cv::getTickCount();
    detector->detect(imgGray, kptsFAST);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "FAST with n= " << kptsFAST.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    cv::Mat visImageFAST = img.clone();
    cv::drawKeypoints(img, kptsFAST, visImageFAST, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    string windowNameFAST = "FAST Results";
    cv::namedWindow(windowNameFAST, 1);
    imshow(windowNameFAST, visImageFAST);

    // BRISK detector
    int thresholdBRISK = 30; // difference between intensity of the central pixel and pixels of a circle around this pixel
    int octaves = 3;         // detection octaves (use 0 to do single scale)
    float patternScale = 1.0f;
    cv::Ptr<cv::FeatureDetector> detectorBRISK = cv::BRISK::create(thresholdBRISK, octaves, patternScale);
    vector<cv::KeyPoint> kptsBRISK;
    t = (double)cv::getTickCount();
    detectorBRISK->detect(imgGray, kptsBRISK);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "BRISK with n= " << kptsBRISK.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
    
    // visualize results
    cv::Mat visImageBRISK = img.clone();
    cv::drawKeypoints(img, kptsBRISK, visImageBRISK, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    string windowNameBRISK = "BRISK Results";
    cv::namedWindow(windowNameBRISK, 1);
    imshow(windowNameBRISK, visImageBRISK);
    
}

int main()
{
    detKeypoints1();
}