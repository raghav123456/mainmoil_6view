/*
 Monocular Video Odometry
 By : Raghav,Shivani
 Guided by: Prof. Cjchang

 */
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <iostream>
#include <ctype.h>
#include <algorithm>               // for copy
#include <iterator>                // for ostream_iterator
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>

using namespace cv;
using namespace std;

#define MAX_FRAME 300
#define MIN_NUM_FEAT 4000
#define PLOT_COLOR CV_RGB(0, 0, 0)
#define PL std::setprecision(3)

/*
 *Vectors in C++ are sequence containers representing arrays that can change in size.
 *Syntax : vector<type> variable_name;
 *double: data type
 *
 */

char *POSES = "/home/raghav/Documents/6_view_Dataset/D5_17-04/D5.txt";
Mat trajBuffer;

vector<double> getAbsoluteScales() {
	vector<double> scales;
	string line;
	double temp;
	ifstream myfile(POSES);    // poses file open while executed

	double x = 0, y = 0, z = 0;
	double x_prev = 0, y_prev = 0, z_prev = 0;
	if (myfile.is_open()) {
		while (getline(myfile, line)) {

			z_prev = z;
			x_prev = x;
			y_prev = y;
			std::istringstream in(line);
//cout << line << '\n';
			for (int j = 0; j < 12; j++) {
				in >> temp;
				if (j == 3)
					x = temp;
				if (j == 7)
					y = temp;
				if (j == 11)
					z = temp;
			}
			/*
			 * push_back is a library function of vector.A vector header it is used to insert/add element at the end of vector
			 * push_back() function is used to push elements into a vector from the back. The new value is inserted into the
			 *  vector at the end,after the current last element and the container size is increased by 1.
			 */
			scales.push_back(
					sqrt(
							(x - x_prev) * (x - x_prev)
									+ (y - y_prev) * (y - y_prev)
									+ (z - z_prev) * (z - z_prev)));
		}
		/*
		 static int timeOpen = 0 ;
		 timeOpen++ ;
		 cout << "OPEN POSE FILE = " << timeOpen ;
		 myfile.close();
		 */
	}
	return scales;
}

void getPosition(string line, float vout[3]) {
// If possible, always prefer std::vector to naked array
	std::vector<float> v;

// Build an istream that holds the input string
	std::istringstream iss(line);

// Iterate over the istream, using >> to grab floats
// and push_back to store them in the vector
	std::copy(std::istream_iterator<float>(iss), std::istream_iterator<float>(),
			std::back_inserter(v));

	vout[0] = v[3];
	vout[1] = v[7];
	vout[2] = v[11];

}

void featureTracking(Mat img_1, Mat img_2, vector<Point2f> &points1,
		vector<Point2f> &points2, vector<uchar> &status)

		{
//this function automatically gets rid of points for which tracking fails

	vector<float> err;
	Size winSize = Size(21, 21);
	TermCriteria termcrit = TermCriteria(
			TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01);

	calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err, winSize,
			3, termcrit, 0, 0.001);

//getting rid of points for which the KLT tracking failed or those who have gone outside the frame
	int indexCorrection = 0;
	for (int i = 0; i < status.size(); i++) {
		Point2f pt = points2.at(i - indexCorrection);
		if ((status.at(i) == 0) || (pt.x < 0) || (pt.y < 0)) {
			if ((pt.x < 0) || (pt.y < 0)) {
				status.at(i) = 0;
			}
			points1.erase(points1.begin() + (i - indexCorrection));
			points2.erase(points2.begin() + (i - indexCorrection));
			indexCorrection++;
		}
	}
}

void featureDetection(Mat img_1, vector<Point2f> &points1, string dir)

{
	int fast_threshold;
//uses FAST as of now, modify parameters as necessary
	vector<KeyPoint> keypoints_1;
// int fast_threshold = 21;
	if (!dir.compare("front")) {
		fast_threshold = 17.7; // 21.8
	} else if (!dir.compare("left")) {
		fast_threshold = 20; // 21.8
	} else if (!dir.compare("right")) {
		fast_threshold = -10; // 21.8
	}

	bool nonmaxSuppression = true;
	FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
	KeyPoint::convert(keypoints_1, points1, vector<int>());

}

int calculateTraj(Mat images_array[300], string direction, Mat trajBufferCopy) {

	bool renderFeatures = false;
	/*
	 if (argc >= 1)
	 {
	 renderFeatures = true ;
	 }
	 */

	Mat img_1, img_2;
	Mat R_f, t_f;     //the final rotation and tranlation vectors containing the

	ofstream myfile;
	myfile.open("result1_1.txt");

	double scale;

	char text[100];
	int fontFace = FONT_HERSHEY_PLAIN;
	double fontScale = 1;
	int thickness = 1;
	cv::Point textOrg(10, 50);

//read the first two frames from the dataset
	Mat img_1_c = images_array[0];// @suppress("Invalid arguments")
	Mat img_2_c = images_array[1];// @suppress("Invalid arguments")

	if (!img_1_c.data || !img_2_c.data)
	{
		std::cout << " --(!) Error reading images " << std::endl;
		return -1;
	}

	string resultFile = "result1_1.txt";

//obtain truth for plot comparison
	string posePath = POSES;
	std::ifstream infile(posePath.c_str());
	std::string line;

	float truthPosition[3];
	Mat truthOrientation;

	getline(infile, line);
	getPosition(line, truthPosition);

// Open a txt file to store the results
	ofstream fout(resultFile.c_str());
	if (!fout)
	{
		cout << "File not opened!" << endl;
		return 1;
	}

// we work with grayscale images
	cvtColor(img_1_c, img_1, COLOR_BGR2GRAY);
	cvtColor(img_2_c, img_2, COLOR_BGR2GRAY);

// feature detection, tracking
	vector<Point2f> points1, points2;//vectors to store the coordinates of the feature points
	featureDetection(img_1, points1,direction);//detect features in img_1
	vector<uchar> status;
	featureTracking(img_1, img_2, points1, points2, status);//track those features to img_2

// WARNING: different sequences in the Dataset have different intrinsic/extrinsic parameters
// Here we use camera parameters of Intel Realsense T65(tracking) camera

	double focal = 303.3202;
	cv::Point2d pp(428.7000, 398.7000);

//recovering the pose and the essential matrix
	Mat E, R, t, mask;
	E = findEssentialMat(points2, points1, focal, pp, RANSAC, 0.999, 1.0, mask);
	recoverPose(E, points2, points1, R, t, focal, pp, mask);

	Mat prevImage = img_2;
	Mat currImage;
	vector<Point2f> prevFeatures = points2;
	vector<Point2f> currFeatures;

	char filename[100];

	R_f = R.clone();
	t_f = t.clone();

	clock_t begin = clock();

	auto groundScales = getAbsoluteScales();
	for (int numFrame = 0; numFrame < MAX_FRAME; numFrame++)		 //numframe starts from 0 because array index [0]
	{

		getline(infile, line);
		getPosition(line, truthPosition);
		Mat currImage_c = images_array[numFrame];
	//	imshow("Road facing camera", currImage_c);

		cvtColor(currImage_c, currImage, COLOR_BGR2GRAY);

		vector<uchar> status;
		featureTracking(prevImage, currImage, prevFeatures, currFeatures,status);

		E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.999, 1.0, mask);
		recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);

		Mat prevPts(2, prevFeatures.size(), CV_64F), currPts(2,currFeatures.size(), CV_64F);

		for (int i = 0; i < prevFeatures.size(); i++)
		{
			//this (x,y) combination makes sense as observed from the source code of triangulatePoints on GitHub
			prevPts.at<double>(0, i) = prevFeatures.at(i) . x;
			prevPts.at<double>(1, i) = prevFeatures.at(i) . y;

			currPts.at<double>(0, i) = currFeatures.at(i) . x;
			currPts.at<double>(1, i) = currFeatures.at(i) . y;
		}

		scale = groundScales[numFrame];

		if ((scale > 0.1 ) && (t.at<double>(2) > t.at<double>(0))&& (t.at<double>(2) > t.at<double>(1)))
		{
			cout << "Scale is " << scale << " " << numFrame<<endl;

			t_f = t_f - scale * R_f * t;
			R_f = R.t() * R_f;

		}
		// a redetection is triggered in case the number of feautres being trakced go below a particular threshold
		if (prevFeatures.size() < MIN_NUM_FEAT)
		{
			featureDetection(prevImage, prevFeatures,direction);
			featureTracking(prevImage, currImage, prevFeatures, currFeatures,status);
		}

		prevImage = currImage.clone();
		prevFeatures = currFeatures;

		int x, y, xTruth, yTruth, x_left, y_left ,x_right, y_right;

		if(!direction.compare("front"))
		{
			x = 2 * int(t_f.at<double>(0)) + 600;
			y = 2 *  int(t_f.at<double>(2)) + 300;
			circle(trajBufferCopy, Point(x, y), 1, CV_RGB(255, 0, 0), 2); //RED
			cout << "COLOR is RED" << endl;

			xTruth = 2 * int(truthPosition[0]) + 600;
			yTruth = 2 * int(truthPosition[2]) + 300;
			circle(trajBufferCopy, Point(xTruth, yTruth), 1, CV_RGB(0, 255, 0), 2);//GREEN

		}
		else if(!direction.compare("left"))
		{

			x_left = 2 * int(t_f.at<double>(0)) + 600;
			y_left = 2 * int(t_f.at<double>(2)) + 300;
			circle(trajBufferCopy, Point(x_left, y_left), 1, CV_RGB(0, 0, 255), 2);//BLUE
			cout << "COLOR is blueeeeee" << endl;
		}
		else if ( !direction.compare ("right") )
		{
			x_right = 2 * int(t_f.at<double>(0)) + 600;
			y_right = 2 * int(t_f.at<double>(2)) + 300;
			circle(trajBufferCopy, Point(x_right, y_right), 1, Scalar(0, 255, 255), 2); //YELLOW

		}

		rectangle(trajBufferCopy, Point(10, 30), Point(550, 50), CV_RGB(0, 0, 0),CV_FILLED);
		if(!direction.compare("front"))
		{
			sprintf(text, "Coordinates: x = %03fm y = %03fm z = %03fm",t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));
		}
		else if(!direction.compare("left"))
		{
			sprintf(text, "Coordinates: x_left = %03fm y_left = %03fm z = %03fm",t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));
		}
		else if(!direction.compare("right"))
		{
			sprintf(text, "Coordinates: x_right = %03fm y_right = %03fm z = %03fm",t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));
		}
		putText(trajBufferCopy, text, textOrg, fontFace, fontScale, Scalar::all(255),thickness, 8);

//Draw features as markers
		if (renderFeatures)
		{
			for (auto point : currFeatures)
			cv::drawMarker(currImage_c, cv::Point(point.x, point.y),CV_RGB(0, 255, 0), cv::MARKER_TILTED_CROSS, 2, 1,cv::LINE_AA);
		}

		imshow("Road facing camera", currImage_c);
		imshow("Trajectory", trajBufferCopy);

		waitKey(1);
	}

	if(!direction.compare("front") || !direction.compare("left"))
	{
		trajBuffer = trajBufferCopy;
	}

// to record the tim that in how much time the trajectory completes
	clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	cout << "Total time taken: " << elapsed_secs << "s" << endl;

// edited to stop the output frame and let user press a key to exit the output window
	myfile.close();

// to save the output trajectory map
	putText(trajBufferCopy, "Monocular VO", Point(250, 900), FONT_HERSHEY_PLAIN, 2,Scalar::all(255), 2, 8);
	sprintf(filename, "/home/raghav/Documents/result/%d.png");
	imwrite(filename, trajBufferCopy);

}
