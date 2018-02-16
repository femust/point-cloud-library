#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <pcl/io/pcd_io.h>


#include <iostream>

using namespace cv;
using namespace std;



static void help(char* progName)
{
    cout << endl
        <<  "This program demonstrated the use of the discrete Fourier transform (DFT). " << endl
        <<  "The dft of an image is taken and it's power spectrum is displayed."          << endl
        <<  "Usage:"                                                                      << endl
        << progName << " [image_name -- default stairs2.jpg] "                       << endl << endl;
}
typedef pcl::PointXYZRGBA PointT;
int main(int argc, char ** argv)
{
    if (argc<2)
    {
        std::cerr << "\033[1;31mGIVE ME A FILE\033[0m\n"<<std::endl;
        return 0;
    }

      pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
      pcl::PCDReader reader;
      reader.read (argv[1], *cloud);
      std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;

cv::Mat image;
image = cv::Mat( cloud->height, cloud->width, CV_8UC4 );


for( int y = 0; y < image.rows; y++ ) {
       for( int x = 0; x < image.cols; x++ ) {
           pcl::PointXYZRGBA point = cloud->at( x, y );
           image.at<cv::Vec4b>( y, x )[0] = point.b;
           image.at<cv::Vec4b>( y, x )[1] = point.g;
           image.at<cv::Vec4b>( y, x )[2] = point.r;
           image.at<cv::Vec4b>( y, x )[3] = point.a;
       }
}

Mat gray_image;
cvtColor( image, gray_image, CV_BGR2GRAY );

imwrite( "Gray_Image.jpg", gray_image );

        //  std::cout << image;


//cv.dft

//    Mat padded;                            //expand input image to optimal size
//    int m = getOptimalDFTSize( image.rows );
//    int n = getOptimalDFTSize( image.cols ); // on the border add zero values
//    copyMakeBorder(image, padded, 0, m - image.rows, 0, n - image.cols, BORDER_CONSTANT, Scalar::all(0));

//  Mat planes[] = {Mat_<float>(padded), Mat::zeros(padded.size(), CV_32F)};
//    Mat complexI;
//    merge(planes, 2, complexI);         // Add to the expanded another plane with zeros

//    dft(complexI, complexI);            // this way the result may fit in the source matrix

//    // compute the magnitude and switch to logarithmic scale
//    // => log(1 + sqrt(Re(DFT(I))^2 + Im(DFT(I))^2))
//    split(complexI, planes);                   // planes[0] = Re(DFT(I), planes[1] = Im(DFT(I))
//    magnitude(planes[0], planes[1], planes[0]);// planes[0] = magnitude
//    Mat magI = planes[0];

//    magI += Scalar::all(1);                    // switch to logarithmic scale
//    log(magI, magI);

//    // crop the spectrum, if it has an odd number of rows or columns
//    magI = magI(Rect(0, 0, magI.cols & -2, magI.rows & -2));

//    // rearrange the quadrants of Fourier image  so that the origin is at the image center
//    int cx = magI.cols/2;
//    int cy = magI.rows/2;

//    Mat q0(magI, Rect(0, 0, cx, cy));   // Top-Left - Create a ROI per quadrant
//    Mat q1(magI, Rect(cx, 0, cx, cy));  // Top-Right
//    Mat q2(magI, Rect(0, cy, cx, cy));  // Bottom-Left
//    Mat q3(magI, Rect(cx, cy, cx, cy)); // Bottom-Right

//    Mat tmp;                           // swap quadrants (Top-Left with Bottom-Right)
//    q0.copyTo(tmp);
//    q3.copyTo(q0);
//    tmp.copyTo(q3);

//    q1.copyTo(tmp);                    // swap quadrant (Top-Right with Bottom-Left)
//    q2.copyTo(q1);
//    tmp.copyTo(q2);

//    normalize(magI, magI, 0, 1, CV_MINMAX); // Transform the matrix with float values into a
//                                            // viewable image form (float between values 0 and 1).

//    imshow("Input Image"       , image   );    // Show the result
//    imshow("spectrum magnitude", magI);
    waitKey();











    return 0;
}
