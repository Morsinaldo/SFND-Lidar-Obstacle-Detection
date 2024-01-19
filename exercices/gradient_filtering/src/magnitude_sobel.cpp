#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;

void magnitudeSobel()
{
    // load image from file
    cv::Mat img;
    img = cv::imread("../images/img1gray.png");

    // check if the image was loaded successfully
    if (img.empty())
    {
        cerr << "Error: Could not load the image." << endl;
        return;
    }
    else
    {
        cout << "Image loaded successfully." << endl;
    }

    // convert image to grayscale
    cv::Mat imgGray;
    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

    // apply smoothing using the GaussianBlur() function from OpenCV
    cv::Mat imgSmoothed;
    cv::GaussianBlur(imgGray, imgSmoothed, cv::Size(5, 5), 0);

    // create filter kernels using the cv::Mat datatype both for x and y
    cv::Mat sobelKernelX = (cv::Mat_<float>(3, 3) << -1, 0, 1, -2, 0, 2, -1, 0, 1);
    cv::Mat sobelKernelY = (cv::Mat_<float>(3, 3) << -1, -2, -1, 0, 0, 0, 1, 2, 1);

    // apply filter using the OpenCV function filter2D()
    cv::Mat gradientX, gradientY;
    cv::filter2D(imgSmoothed, gradientX, -1, sobelKernelX);
    cv::filter2D(imgSmoothed, gradientY, -1, sobelKernelY);

    // check if gradient images are empty and have the same size
    if (gradientX.empty() || gradientY.empty() || gradientX.size() != gradientY.size())
    {
        cerr << "Error: Gradient images have different sizes." << endl;
        return;
    }

    // convert the gradient images to a common type (CV_64F)
    gradientX.convertTo(gradientX, CV_64F);
    gradientY.convertTo(gradientY, CV_64F);

    // compute magnitude image based on the equation presented in the lesson
    cv::Mat magnitude;
    try
    {
        cv::magnitude(gradientX, gradientY, magnitude);

        // Normalize the magnitude image for better visualization
        cv::normalize(magnitude, magnitude, 0, 255, cv::NORM_MINMAX);
    }
    catch (cv::Exception &e)
    {
        cerr << "OpenCV Exception: " << e.what() << endl;
        return;
    }

    // show result
    string windowName = "Sobel Magnitude";
    cv::namedWindow(windowName, 1); // create window
    cv::imshow(windowName, magnitude);
    cv::waitKey(0); // wait for keyboard input before continuing
}

int main()
{
    magnitudeSobel();
    return 0;
}
