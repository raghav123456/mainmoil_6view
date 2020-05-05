#include "car6view.h"
#include "vo.h"
#include <stdio.h>
#include <string>
#include <string.h>
#include <iostream>

#define MAP_CACHE_ENABLED true
#define USE_PICAMERA true
#define MAX_FRAME 300
#define MIN_NUM_FEAT 4000
#define PLOT_COLOR CV_RGB(0, 0, 0)
#define PL std::setprecision(3)

using namespace cv;
using namespace std;
string direction;

char *IMAGES = "/home/raghav/Documents/6_view_Fisheyeimages/D5/";

Mat all_front_images[300];                   // array size defined
Mat all_left_images[300];
Mat all_right_images[300];
//Mat all_down_images[500];
//Mat all_lower_left_images[500];
//Mat all_lower_right_images[500];

char filename[100];

Car6view::Car6view() {
	md = new Moildev();
}

string convertToString(char *a, int size) // char array ko string me convert karta haii
		{
	int i;
	string s = "";
	for (i = 0; i < size; i++) {
		s = s + a[i];
	}
	return s;
}

void Car6view::Show() {

	//Intel T265 parameters:21stMarch-2020
	md->Config("car", 3, 3,
			427, 394, 1, //center
			848, 800, 1.68, // resolution
			0, 0, -24.964, 38.2, -16.956, 183.42 //calibration
			);

	double calibrationWidth = md->getImageWidth();
	double iCy = md->getiCy();
	ConfigData *cd = md->getcd();
	//char filename[100];

	image_input = imread("/home/raghav/Documents/6_view_Fisheyeimages/D5/1.png",
			IMREAD_COLOR);

	MediaType mediaType = MediaType::IMAGE_FILE;
	double w = image_input.cols;
	double h = image_input.rows;

	mapX[0] = Mat(h, w, CV_32F);
	mapX[1] = Mat(w, h, CV_32F);
	mapX[2] = Mat(w, h, CV_32F);
	mapY[0] = Mat(h, w, CV_32F);
	mapY[1] = Mat(w, h, CV_32F);
	mapY[2] = Mat(w, h, CV_32F);

	for (uint i = 0; i < 7; i++)
		mapX[i] = Mat(h, w, CV_32F);
	for (uint i = 0; i < 7; i++)
		mapY[i] = Mat(h, w, CV_32F);

	Mat image_result(h, w, CV_32F);
	Mat image_resultv(w, h, CV_32F);
	m_ratio = w / calibrationWidth;
	clock_t tStart = clock();
	char str_x[12], str_y[12];
	int i = 0;
	if ( MAP_CACHE_ENABLED) {

		bool map_exist = true;

		while (map_exist && (i < 7)) {
			sprintf(str_x, "matX%d", i);
			sprintf(str_y, "matY%d", i);
			if (!fopen(str_x, "r") || !fopen(str_y, "r"))
				map_exist = false;
			i++;
		}
		if (map_exist) {
			for (i = 0; i < 7; i++) {
				sprintf(str_x, "matX%d", i);
				sprintf(str_y, "matY%d", i);
				mapX[i] = MatRead(str_x);
				mapY[i] = MatRead(str_y);
			}
		} else {
			md->AnyPointM((float*) mapX[0].data, (float*) mapY[0].data,
					mapX[0].cols, mapX[0].rows, 0, 0, 4, m_ratio); // front view
			md->AnyPointM((float*) mapX[1].data, (float*) mapY[1].data,
					mapX[1].cols, mapX[1].rows, 30, 270, 4, m_ratio); // left view, rotate 90
			md->AnyPointM((float*) mapX[2].data, (float*) mapY[2].data,
					mapX[2].cols, mapX[2].rows, 30, 90, 4, m_ratio); // right view, rotate -90
			md->AnyPointM((float*) mapX[3].data, (float*) mapY[3].data,
					mapX[3].cols, mapX[3].rows, -30, 0, 4, m_ratio); // Down view ( zoom: 2/4 )
			md->AnyPointM((float*) mapX[4].data, (float*) mapY[4].data,
					mapX[4].cols, mapX[4].rows, 30, 225, 4, m_ratio); // left-lower view, rotate 180
			md->AnyPointM((float*) mapX[5].data, (float*) mapY[5].data,
					mapX[5].cols, mapX[5].rows, 30, 135, 4, m_ratio); // right-lower view, rotate 180
			md->PanoramaM((float*) mapX[6].data, (float*) mapY[6].data,
					mapX[6].cols, mapX[6].rows, m_ratio, 110);   // panorama
			for (i = 0; i < 7; i++) {
				sprintf(str_x, "matX%d", i);
				sprintf(str_y, "matY%d", i);
				MatWrite(str_x, mapX[i]);
				MatWrite(str_y, mapY[i]);
			}
		}
	} else {
		md->AnyPointM((float*) mapX[0].data, (float*) mapY[0].data,
				mapX[0].cols, mapX[0].rows, 0, 0, 4, m_ratio);     // front view
		md->AnyPointM((float*) mapX[1].data, (float*) mapY[1].data,
				mapX[1].cols, mapX[1].rows, 30, 270, 4, m_ratio); // left view, rotate 90
		md->AnyPointM((float*) mapX[2].data, (float*) mapY[2].data,
				mapX[2].cols, mapX[2].rows, 30, 90, 4, m_ratio); // right view, rotate -90
		md->AnyPointM((float*) mapX[3].data, (float*) mapY[3].data,
				mapX[3].cols, mapX[3].rows, -30, 0, 4, m_ratio); // Down view ( zoom: 2/4 )
		md->AnyPointM((float*) mapX[4].data, (float*) mapY[4].data,
				mapX[4].cols, mapX[4].rows, 30, 225, 4, m_ratio); // left-lower view, rotate 180
		md->AnyPointM((float*) mapX[5].data, (float*) mapY[5].data,
				mapX[5].cols, mapX[5].rows, 30, 135, 4, m_ratio); // right-lower view, rotate 180
		md->PanoramaM((float*) mapX[6].data, (float*) mapY[6].data,
				mapX[6].cols, mapX[6].rows, m_ratio, 110);   // panorama
	}

	double time_clock = (double) (clock() - tStart) / CLOCKS_PER_SEC;
	cout << "time: " << time_clock << endl;
	Vec3b p(0, 0, 0);
	image_input.at<Vec3b>(0, 0) = p;

	string image_name = "1.png";

	DisplayCh(0, image_name, 0); // img-display CALL
	char c; //int
	//char getch(void);
	// yaha while loop me condition tre nh ho rhi jiske krn code chlte ja rhe again-gaian.
	while (1) {
		c = waitKey(1000);
		if (c == 27)
			break;
		//cout << "press o if you want to call opencamera" << endl;
	//	char o;
		//o = getch();
		//cout << o << endl;
		//if (c == 'c' || o == 'o') {
		if (c == 'c'){
			cout << "open camera " << endl;
			openCamara();
		}
	}

	cvDestroyWindow("image_input");
	cvDestroyWindow("Front");
	cvDestroyWindow("Left");
	cvDestroyWindow("Right");
	cvDestroyWindow("Down");
	cvDestroyWindow("Lower left");
	cvDestroyWindow("Lower right");
	image_result.release();
	image_resultv.release();
}

void Car6view::DisplayCh(int ch, string image_name2, int count)  // @shivani -
		{
	//cout<<"we are in dislplaych method"<<endl;

	Mat image_result, image_resultv;
	if (image_input.empty())
		return;
	if (currCh != prevCh) {
		cvDestroyWindow("image_input");
		cvDestroyWindow("Front");
		cvDestroyWindow("Left");
		cvDestroyWindow("Right");
		cvDestroyWindow("Down");
		cvDestroyWindow("Lower left");
		cvDestroyWindow("Lower right");

	}
    Mat temp_img;
	// original image
	switch (ch) {
	case 0:  // 2 x 3

		cv::resize(image_input, image_input_s,
				Size(width_split, height_split - y_base));
		if ((mediaType == MediaType::CAMERA) && USE_PICAMERA)
			cvtColor(image_input_s, image_input_s, CV_BGR2RGB);
		//imshow("image_input", image_input_s);
		moveWindow("image_input", x_base + width_split, 0 + y_base);
		remap(image_input, image_result, mapX[0], mapY[0], INTER_CUBIC,
				BORDER_CONSTANT, Scalar(0, 0, 0));

		cv::resize(image_result, image_display[0],
				Size(width_split, height_split - y_base));
		if ((mediaType == MediaType::CAMERA) && USE_PICAMERA)
			cvtColor(image_display[0], image_display[0], CV_BGR2RGB);
		//all_front_images[count] = image_display[0];
	    //    temp_img = image_display[0];
		//all_front_images[count] = temp_img;
		cout<<"print displaych"<<endl;
		imshow("Front", image_display[0]);
		cout<<"print image disply ="  << endl;
		cout<<"print value of count"<<count<<endl;
		image_display[0].copyTo(all_front_images[count]);  //changed
		cout<<"print value of count="<<count<<endl;
	//	imshow("Front", image_display[0]);
	//	imshow("Front",image_display[0]);
		moveWindow("Front", x_base + width_split, 0 + y_base);

		remap(image_input, image_resultv, mapX[1], mapY[1], INTER_CUBIC,
				BORDER_CONSTANT, Scalar(0, 0, 0));
		Rotate(image_resultv, image_result, 90.0);
		cv::resize(image_result, image_display[1],
				Size(width_split, height_split - y_base));
		if ((mediaType == MediaType::CAMERA) && USE_PICAMERA)
			cvtColor(image_display[1], image_display[1], CV_BGR2RGB);
	//	imshow("Left", image_display[1]);
		image_display[1].copyTo(all_left_images[count]);  //changed
		moveWindow("Left", x_base, 0 + y_base);

		remap(image_input, image_resultv, mapX[2], mapY[2], INTER_CUBIC,
				BORDER_CONSTANT, Scalar(0, 0, 0));
		Rotate(image_resultv, image_result, -90.0);
		cv::resize(image_result, image_display[2],
				Size(width_split, height_split - y_base));
		if ((mediaType == MediaType::CAMERA) && USE_PICAMERA)
			cvtColor(image_display[2], image_display[2], CV_BGR2RGB);
	//	imshow("Right", image_display[2]);
		image_display[2].copyTo(all_right_images[count]);  //changed
		moveWindow("Right", x_base + width_split * 2, 0 + y_base);

		remap(image_input, image_result, mapX[3], mapY[3], INTER_CUBIC,
				BORDER_CONSTANT, Scalar(0, 0, 0));
		cv::resize(image_result, image_display[3],
				Size(width_split, height_split - y_base));
		if ((mediaType == MediaType::CAMERA) && USE_PICAMERA)
			cvtColor(image_display[3], image_display[3], CV_BGR2RGB);
	//	imshow("Down", image_display[3]);
		moveWindow("Down", x_base + width_split, height_split + y_base);

		remap(image_input, image_result, mapX[4], mapY[4], INTER_CUBIC,
				BORDER_CONSTANT, Scalar(0, 0, 0));
		Rotate(image_result, image_result, 180.0);
		cv::resize(image_result, image_display[4],
				Size(width_split, height_split - y_base));
		if ((mediaType == MediaType::CAMERA) && USE_PICAMERA)
			cvtColor(image_display[4], image_display[4], CV_BGR2RGB);
	//	imshow("Lower left", image_display[4]);
		moveWindow("Lower left", x_base, height_split + y_base);

		remap(image_input, image_result, mapX[5], mapY[5], INTER_CUBIC,
				BORDER_CONSTANT, Scalar(0, 0, 0));
		Rotate(image_result, image_result, 180.0);
		cv::resize(image_result, image_display[5],
				Size(width_split, height_split - y_base));
		if ((mediaType == MediaType::CAMERA) && USE_PICAMERA)
			cvtColor(image_display[5], image_display[5], CV_BGR2RGB);
	//	imshow("Lower right", image_display[5]);
		moveWindow("Lower right", x_base + width_split * 2,
				height_split + y_base);
		break;

	case 1:
		cv::resize(image_input, image_input_s,
				Size(width_split * 3, height_split * 2));
		if ((mediaType == MediaType::CAMERA) && USE_PICAMERA)
			cvtColor(image_input_s, image_input_s, CV_BGR2RGB);
		imshow("image_input_var", image_input_s);
		moveWindow("image_input", x_base, y_base);
		break;
	case 2:
		remap(image_input, image_result, mapX[0], mapY[0], INTER_CUBIC,
				BORDER_CONSTANT, Scalar(0, 0, 0));
		cv::resize(image_result, image_display[0],
				Size(width_split * 3, height_split * 2));
		if ((mediaType == MediaType::CAMERA) && USE_PICAMERA)
			cvtColor(image_display[0], image_display[0], CV_BGR2RGB);
		imshow("Front", image_display[0]);
		moveWindow("Front", x_base, y_base);
		break;
	case 3:
		remap(image_input, image_resultv, mapX[1], mapY[1], INTER_CUBIC,
				BORDER_CONSTANT, Scalar(0, 0, 0));
		Rotate(image_resultv, image_result, 90.0);
		cv::resize(image_result, image_display[1],
				Size(width_split * 3, height_split * 2));
		if ((mediaType == MediaType::CAMERA) && USE_PICAMERA)
			cvtColor(image_display[1], image_display[1], CV_BGR2RGB);
		imshow("Left", image_display[1]);
		moveWindow("Left", x_base, y_base);
		break;
	case 4:
		remap(image_input, image_resultv, mapX[2], mapY[2], INTER_CUBIC,
				BORDER_CONSTANT, Scalar(0, 0, 0));
		Rotate(image_resultv, image_result, -90.0);
		cv::resize(image_result, image_display[2],
				Size(width_split * 3, height_split * 2));
		if ((mediaType == MediaType::CAMERA) && USE_PICAMERA)
			cvtColor(image_display[2], image_display[2], CV_BGR2RGB);
		imshow("Right", image_display[2]);
		moveWindow("Right", x_base, y_base);
		break;
	case 5:
		remap(image_input, image_result, mapX[3], mapY[3], INTER_CUBIC,
				BORDER_CONSTANT, Scalar(0, 0, 0));
		cv::resize(image_result, image_display[3],
				Size(width_split * 3, height_split * 2));
		if ((mediaType == MediaType::CAMERA) && USE_PICAMERA)
			cvtColor(image_display[3], image_display[3], CV_BGR2RGB);
		imshow("Down", image_display[3]);
		moveWindow("Down", x_base, y_base);
		break;
	case 6:
		remap(image_input, image_result, mapX[4], mapY[4], INTER_CUBIC,
				BORDER_CONSTANT, Scalar(0, 0, 0));
		Rotate(image_result, image_result, 180.0);
		cv::resize(image_result, image_display[4],
				Size(width_split * 3, height_split * 2));
		if ((mediaType == MediaType::CAMERA) && USE_PICAMERA)
			cvtColor(image_display[4], image_display[4], CV_BGR2RGB);
		imshow("Lower left", image_display[4]);
		moveWindow("Lower left", x_base, y_base);
		break;
	case 7:
		remap(image_input, image_result, mapX[5], mapY[5], INTER_CUBIC,
				BORDER_CONSTANT, Scalar(0, 0, 0));
		Rotate(image_result, image_result, 180.0);
		cv::resize(image_result, image_display[5],
				Size(width_split * 3, height_split * 2));
		if ((mediaType == MediaType::CAMERA) && USE_PICAMERA)
			cvtColor(image_display[5], image_display[5], CV_BGR2RGB);
		imshow("Lower right", image_display[5]);
		moveWindow("Lower right", x_base, y_base);
		break;
	case 8:
		remap(image_input, image_result, mapX[6], mapY[6], INTER_CUBIC,
				BORDER_CONSTANT, Scalar(0, 0, 0));
		cv::resize(image_result, image_display[6],
				Size(width_split * 3, height_split * 2));
		if ((mediaType == MediaType::CAMERA) && USE_PICAMERA)
			cvtColor(image_display[6], image_display[6], CV_BGR2RGB);
		imshow("Panorama", image_display[6]);
		moveWindow("Panorama", x_base, y_base);
		break;
	case -1:
		cout << "test" << endl;
		break;
	}
	prevCh = currCh;
}

void Car6view::Rotate(Mat &src, Mat &dst, double angle) {
	Point2f center((src.cols - 1) / 2.0, (src.rows - 1) / 2.0);
	Mat rot = cv::getRotationMatrix2D(center, angle, 1.0);
	Rect2f bbox =
			cv::RotatedRect(cv::Point2f(), src.size(), angle).boundingRect2f();
	rot.at<double>(0, 2) += bbox.width / 2.0 - src.cols / 2.0;
	rot.at<double>(1, 2) += bbox.height / 2.0 - src.rows / 2.0;
	warpAffine(src, dst, rot, bbox.size());
}

void Car6view::MatWrite(const string &filename, const Mat &mat) {
	ofstream fs(filename, fstream::binary);

	// Header
	int type = mat.type();
	int channels = mat.channels();
	fs.write((char*) &mat.rows, sizeof(int));    // rows
	fs.write((char*) &mat.cols, sizeof(int));    // cols
	fs.write((char*) &type, sizeof(int));        // type
	fs.write((char*) &channels, sizeof(int));    // channels

	// Data
	if (mat.isContinuous()) {
		fs.write(mat.ptr<char>(0), (mat.dataend - mat.datastart));
	} else {
		int rowsz = CV_ELEM_SIZE(type) * mat.cols;
		for (int r = 0; r < mat.rows; ++r) {
			fs.write(mat.ptr<char>(r), rowsz);
		}
	}
}

Mat Car6view::MatRead(const string &filename) {
	ifstream fs(filename, fstream::binary);

	// Header
	int rows, cols, type, channels;
	fs.read((char*) &rows, sizeof(int));         // rows
	fs.read((char*) &cols, sizeof(int));         // cols
	fs.read((char*) &type, sizeof(int));         // type
	fs.read((char*) &channels, sizeof(int));     // ch


	// Data
	Mat mat(rows, cols, type);
	fs.read((char*) mat.data, CV_ELEM_SIZE(type) * rows * cols);

	return mat;
}

void Car6view::camButtonClicked() {

}

void Car6view::openCamara() {
	cout << "opencamera works " << endl;
	char c;

	if (true) {
		mediaType = MediaType::CAMERA;
		currCh = 0;

		for (int numFrame = 2; numFrame <= 300; numFrame++)           //@Shivani
				{
			sprintf(filename, "%s%d.png", IMAGES, numFrame);
		//	cout<<" value of numFame =" << numFrame<< endl;
			string int_to_string = to_string(numFrame);
			string image_name = int_to_string.append(".png");
			cout << "image name is " << image_name<<endl;

			image_input = imread(filename, IMREAD_COLOR); //@Shivani
	//		cout<<"value of image_input = "<< image_input <<endl;
			if (image_input.empty()){
			cout<<"empty value......."<<endl;
				break; // end of video stream
			}
			cout<<"Displych is woking"<<endl;
			DisplayCh(currCh, image_name, numFrame - 1);   //@Shivani
			c = waitKey(1);

		}
		mediaType = MediaType::NONE;
		namedWindow("Road facing camera", WINDOW_AUTOSIZE);
		namedWindow("Road facing camera", WINDOW_AUTOSIZE); // Create a window for display.
	//	namedWindow("front", WINDOW_AUTOSIZE);
	//	namedWindow("left", WINDOW_AUTOSIZE);
	//	namedWindow("right", WINDOW_AUTOSIZE);
		namedWindow("Trajectory", WINDOW_AUTOSIZE); // Create a window for display.
		moveWindow("Trajectory", 600, 600);

	Mat traj = Mat::zeros(1000, 1000, CV_8UC3);

		calculateTraj(all_front_images, "front", traj);
		calculateTraj(all_left_images, "left", trajBuffer);
		calculateTraj(all_right_images, "right", trajBuffer);

	}
}

void Car6view::readFarme() {

}

void Car6view::takingPictures() {

}

void Car6view::closeCamara() {

}

void Car6view::freeMemory() {
	cout << "free memory" << endl;
	image_input.release();
	image_input_s.release();
	for (int i = 0; i < 7; i++) {
		image_display[i].release();
		mapX[i].release();
		mapY[i].release();
	}
}
Car6view::~Car6view() {
	freeMemory();
	delete md;
}
