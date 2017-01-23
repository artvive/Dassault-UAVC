#include <iostream>
#include <ctime>
#include <fstream>
#include <string>
#include <zmq.hpp>


#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
//#include "zmq.h"

#include "Arrow.h"
#include "Square.h"
#include "Cross.h"

#define FPS 30
#define ZMQ_UPDATE_INTERVAL 0.3

using namespace cv;
using namespace std;

enum {SQUARE = 0, CROSS = 1, ARROW = 2, GRASS=3};


struct ColorInterval {
   int lowH;
   int highH;
   int lowS;
   int highS;
   int lowV;
   int highV;
   int algo;//0 -> déterministe 1 -> Statistique
};


struct lengthIndex {
   float length;
   int index;
};

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


bool compareLengthIndex(const struct lengthIndex a, const struct lengthIndex b);
bool compareLengthIndex2(const struct lengthIndex a, const struct lengthIndex b);
bool isArrow(const vector<Point>& polygons, float epsilon,Mat& img,const vector<Point>& contour, vector<Arrow> &arrows);
bool isSquare(const vector<Point>& polygons, float epsilon,Mat& img, const vector<Point>& contour, vector<Square> &squares);
bool isCross(const vector<Point>& polygons, float epsilon, Mat& img, const vector<Point>& contour, vector<Cross> &crosses);
void computeFECorrection(Size siz, double alpha, vector<vector<Point> >& correct);
void computeFECorrection2(Size siz, double alpha, vector<vector<Point> >& correct);
void correctFishEye(Mat& src, Mat& dst, const vector<vector<Point> >& correct);
void correctFishEye2(Mat& src, Mat& dst, const vector<vector<Point> >& correct);
void correctFishEye3(Mat& src, Mat& dst, unsigned int data[]);
void correctToData(const vector<vector<Point> >& correct, unsigned int data[], int chan, int col);
void PCAShape(vector<Point> polygons, Point& center, Point& point1, Point& point2 );
void furthestLeftRight(Mat& src, Point& pointLeft, Point& pointRight);
static void objectCallback(int index, void* userdata);
static void lowHCallback(int level, void* userdata);
static void highHCallback(int level, void* userdata);
static void lowSCallback(int level, void* userdata);
static void highSCallback(int level, void* userdata);
static void lowVCallback(int level, void* userdata);
static void highVCallback(int level, void* userdata);
static void algoCallback(int level, void* userdata);

struct ColorInterval objectColor[4];
int activeObject=0;

int syntax(){
    cout << "Syntax error :"<<endl;
    cout << "Examples : shaperecon -v -i test.avi" <<endl;
    cout << "shaperecon -p -i test.png" <<endl;
    cout << "shaperecon -c 0" <<endl;
    cout << "" <<endl;
    return 0;
}

 int main( int argc, char** argv )
 {
    Mat imgOriginal;
    VideoCapture cap;
    VideoWriter oVideoWriter;
    Size frameSize;
    bool isRecording=false, paused=false;
    bool camera=true;
    bool file=true;
    char* inputFile=NULL;
    char* outputFile=NULL;
    int numCam=0;
    bool answerNeeded=false;
    double tolereratedError=0.3;

    for(int i=1;i<argc;i++){

        if(argv[i][0]=='-'){
            if(strlen(argv[i])!=2){
                return syntax();
            }
            switch(argv[i][1]){
                case 'v':
                    camera=true;
                    file=true;
                    break;
                case 'p':
                    camera=false;
                    break;
                case 'c':
                    if(i+1>=argc)
                        return syntax();
                    numCam=argv[i+1][0]-'0';//conversion en int
                    camera=true;
                    file=false;
                    break;
                case 'i':
                    if(i+1>=argc)
                        return syntax();
                    inputFile=argv[i+1];
                    break;
                case 'd':
                    tolereratedError=(double) (argv[i+1][0]-'0')/100.;//erreur en pourcents


            }
        }
    }



     if(!camera){
        if(inputFile==NULL)
            imgOriginal = imread("SquareMesure2.png");
        else
            imgOriginal = imread(inputFile);

        if ( !imgOriginal.data )  // if not success, exit program
        {
             cout << "Cannot open the img" << endl;
             return -1;
        }
        frameSize=imgOriginal.size();

     }
     else{
         //capture the video from webcam
        if(not file){
            cap=VideoCapture(numCam);
        }
        else{
            if(inputFile==NULL)
                cap=VideoCapture("D:/test fleche/HexaTeam1443363144.avi");//HexaTeam1431612466bien.avi//HexaTeam1443703632.avi
            else
                cap=VideoCapture(inputFile);
        }

        if ( !cap.isOpened() )  // if not success, exit program
        {
             cout << "Cannot open the web cam" << endl;
             return -1;
        }
        double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
        double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video
        cout << dWidth<< " "<<dHeight<<endl;

        frameSize=Size(static_cast<int>(dWidth), static_cast<int>(dHeight));
        cap.read(imgOriginal);
        cout << imgOriginal.cols << endl;
        cout << "fps de la cam :"<< cap.get(CV_CAP_PROP_FPS)<<endl;
     }

     ifstream colorFileI("objectColor.txt");
     if(colorFileI){
        string ligne;
        getline(colorFileI, ligne);

        // Create a buffer of the correct length:
        string values;
        int posDel;

        for(int i=0;i<4;i++){
            posDel=ligne.find(";");
            values=ligne.substr(0, posDel);
            ligne.erase(0, posDel + 1);
            objectColor[i].lowH=atoi(values.c_str());
            cout<< values<<endl;
            posDel=ligne.find(";");
            values=ligne.substr(0, posDel);
            ligne.erase(0, posDel + 1);
            objectColor[i].lowS=atoi(values.c_str());
            posDel=ligne.find(";");
            values=ligne.substr(0, posDel);
            ligne.erase(0, posDel + 1);
            objectColor[i].lowV=atoi(values.c_str());
            posDel=ligne.find(";");
            values=ligne.substr(0, posDel);
            ligne.erase(0, posDel + 1);
            objectColor[i].highH=atoi(values.c_str());
            posDel=ligne.find(";");
            values=ligne.substr(0, posDel);
            ligne.erase(0, posDel + 1);
            objectColor[i].highS=atoi(values.c_str());
            posDel=ligne.find(";");
            values=ligne.substr(0, posDel);
            ligne.erase(0, posDel + 1);
            objectColor[i].highV=atoi(values.c_str());
            posDel=ligne.find(";");
            values=ligne.substr(0, posDel);
            ligne.erase(0, posDel + 1);
            objectColor[i].algo=atoi(values.c_str());
        }
     }
     else{
            cout << "impossible d'ouvrir le fichier de Couleurs"<<endl;
        for(int i=0;i<3;i++){
            objectColor[i].lowH=0;
            objectColor[i].highH=179;
            objectColor[i].lowS=0;
            objectColor[i].highS=255;
            objectColor[i].lowV=0;
            objectColor[i].highV=255;
            objectColor[i].algo=0;
        }

        objectColor[GRASS].lowH=0;
        objectColor[GRASS].highH=0;
        objectColor[GRASS].lowS=0;
        objectColor[GRASS].highS=0;
        objectColor[GRASS].lowV=0;
        objectColor[GRASS].highV=0;
        objectColor[GRASS].algo=0;// on s'en fout pour l'herbe
     }



    //ZMQ serveur

    zmq::context_t context (1);
    zmq::socket_t socket (context, ZMQ_REP);
    socket.bind ("tcp://*:5555");


    namedWindow("Control", CV_WINDOW_NORMAL); //create a window called "Control"
    namedWindow("Control 2", CV_WINDOW_NORMAL);

    namedWindow("Thresholded Image", CV_WINDOW_AUTOSIZE);
    namedWindow("No fish Eye", CV_WINDOW_AUTOSIZE);
    namedWindow("Original", CV_WINDOW_AUTOSIZE);

    int iLowH = 0;
    int iHighH = 179;

    int iLowS = 0;
    int iHighS = 255;

    int iLowV = 0;
    int iHighV = 255;

    int iAlgo=0;

    int indexForme=0;


    //Create trackbars in "Control" window
    createTrackbar("Object", "Control", &indexForme, 3,objectCallback);//{CARRE = 0, CROIX = 1, FLECHE = 2, GRASS=3}
    createTrackbar("LowH", "Control", &iLowH, 179,lowHCallback); //Hue (0 - 179)
    createTrackbar("HighH", "Control", &iHighH, 179,highHCallback);

    createTrackbar("LowS", "Control", &iLowS, 255,lowSCallback); //Saturation (0 - 255)
    createTrackbar("HighS", "Control", &iHighS, 255,highSCallback);

    createTrackbar("LowV", "Control", &iLowV, 255,lowVCallback);//Value (0 - 255)
    createTrackbar("HighV", "Control", &iHighV, 255,highVCallback);

    createTrackbar("Deterministe / Statistique", "Control", &iAlgo, 1,algoCallback);

    double alpha=473./1000*1.5;
    int alphaI=473;
    int alphaIOld=473;
    if((numCam==0 && camera && not file)|| not camera){// si Webcam : pas de correction FE
        alpha=1./1000*1.5;
        alphaI=1;
        alphaIOld=1;
    }
    int tolerance=25;
    int aireMini=500;
    createTrackbar("alpha", "Control 2", &alphaI, 1000);
    createTrackbar("tolerance", "Control 2", &tolerance, 100);
    createTrackbar("aire minimale", "Control 2", &aireMini, 2000);

    int iLastX = -1;
    int iLastY = -1;

  //Create a black image with the size as the camera output
    Mat imgLines = Mat::zeros( imgOriginal.size(), CV_8UC3 );

    double currentTime=0;
    double previousTime=(double) clock()/CLOCKS_PER_SEC;

    double zmqPreviousTime=(double) clock()/CLOCKS_PER_SEC;

    vector<Shape> arrows;
    vector<Shape> squares;
    vector<Shape> crosses;

    vector <vector<Shape> > shapes;

    shapes.push_back(squares);
    shapes.push_back(crosses);
    shapes.push_back(arrows);

    vector<vector<Point> > correctFE=vector<vector<Point> >(imgOriginal.size().height,vector<Point>(imgOriginal.size().width,Point(0,0)));
    //vector<unsigned int> dataCorrectFE=vector<unsigned int>(imgOriginal.channels()*(imgOriginal.cols*(imgOriginal.size().height-1)+(imgOriginal.size().width-1))+3,0);

    unsigned int* dataCorrectFE = new unsigned int[imgOriginal.channels()*(imgOriginal.cols*(imgOriginal.size().height-1)+(imgOriginal.size().width-1))+3];

    computeFECorrection2(imgOriginal.size(),alpha,correctFE);
    //computeFECorrection(imgOriginal.size(),alpha,correctFE);
    correctToData(correctFE,dataCorrectFE,imgOriginal.channels(),imgOriginal.cols);

    double deltaT=0;
    int deltaTms=0;

    while (true)
    {
        Mat canny_output;
        vector<vector<Point> > contours;
        vector<Point> polygons;
        vector<Vec4i> hierarchy;
        int thresh= 100;

        imgLines = Mat::zeros( imgOriginal.size(), CV_8UC3 );

        Mat imgHSV,imgScr;

        if(camera && not paused){

            if (!cap.read(imgOriginal)) //if not success, break loop
            {
                cout << "Signal Lost !"<<endl;
                break;
            }
        }

        if(isRecording)
             oVideoWriter.write(imgOriginal);

        //Mat imgNoFE=Mat::zeros(imgOriginal.size(),CV_8UC3);
        Mat imgNoFE=Mat(imgOriginal.size(),CV_8UC3,Scalar((objectColor[GRASS].highH-objectColor[GRASS].lowH)/2,(objectColor[GRASS].highS-objectColor[GRASS].lowS)/2,(objectColor[GRASS].highV-objectColor[GRASS].lowV)/2));

        //correctFishEye2(imgOriginal,imgNoFE,correctFE);
        correctFishEye3(imgOriginal,imgNoFE,dataCorrectFE);
        //imgNoFE=imgOriginal;
        medianBlur( imgNoFE, imgNoFE, 5);//pour enlever les points noir et aplanir les couleurs
        //GaussianBlur(imgNoFE,imgNoFE,Size(5,5),0,0);

        cvtColor(imgNoFE, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

        Mat imgMaskGrass;
        inRange(imgHSV, Scalar(objectColor[GRASS].lowH, objectColor[GRASS].lowS, objectColor[GRASS].lowV), Scalar(objectColor[GRASS].highH, objectColor[GRASS].highS, objectColor[GRASS].highV), imgMaskGrass);

        int levelMorph=5, levelPoly=5;

        // ???????


        //morphological opening (removes small objects from the foreground)
        Mat disque=getStructuringElement(MORPH_ELLIPSE, Size(levelMorph, levelMorph));
        erode(imgMaskGrass, imgMaskGrass, disque );
        dilate( imgMaskGrass, imgMaskGrass, disque );

        //morphological closing (removes small holes from the foreground)
        dilate( imgMaskGrass, imgMaskGrass, disque );
        erode(imgMaskGrass, imgMaskGrass, disque );

        vector<Arrow> newArrows;
        vector<Square> newSquares;
        vector<Cross> newCrosses;




        for(int shape=0;shape<3;shape++){

            Mat imgThresholded;

            inRange(imgHSV, Scalar(objectColor[shape].lowH, objectColor[shape].lowS, objectColor[shape].lowV), Scalar(objectColor[shape].highH, objectColor[shape].highS, objectColor[shape].highV), imgThresholded); //Threshold the image
            imgThresholded=imgThresholded-imgMaskGrass;


            //morphological opening (removes small objects from the foreground)
            Mat disque=getStructuringElement(MORPH_ELLIPSE, Size(levelMorph, levelMorph));
            erode(imgThresholded, imgThresholded, disque );
            dilate( imgThresholded, imgThresholded, disque );

            //morphological closing (removes small holes from the foreground)
            dilate( imgThresholded, imgThresholded, disque );
            erode(imgThresholded, imgThresholded, disque );

            //Canny( imgThresholded, canny_output, thresh, thresh*2, 3 );
            Mat copyImgThresholded=imgThresholded.clone();
            findContours( copyImgThresholded, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
            //drawContours(imgLines, contours, -1, Scalar(0,0,255));

            //cout << "nb de contours :" << contours.size()<<endl;


            for (int i=0;i<contours.size();i++){
                approxPolyDP(contours[i], polygons, levelPoly, true);
                if(polygons.size()>=3 && contourArea(contours[i])>aireMini){
                    Scalar color=Scalar(255,0,0);
                    int thickness=2;
                    if(shape==ARROW && polygons.size()>=4){

                        if(objectColor[ARROW].algo==1){

                            color=Scalar(0,255,0);
                            Point directArrow;

                            Point center,point1,point2;
                            PCAShape(polygons,center,point1,point2);
                            circle(imgLines, center, 3, Scalar(255, 0, 255), 2);
                            line(imgLines, center, point1, Scalar(0, 255, 0), 2);
                            line(imgLines, center, point2, Scalar(255, 255, 0), 2);

                            Mat imgContours = Mat::zeros( imgOriginal.size(), CV_8UC1 );
                            Mat imgAxis = Mat::zeros( imgOriginal.size(), CV_8UC1);

                            line(imgAxis, center, point1+10*(point1-center), Scalar(255), 2);
                            line(imgAxis, center, point1-10*(point1-center), Scalar(255), 2);
                            drawContours(imgContours,contours,i,Scalar(255),2);
                            Mat result=imgContours & imgAxis;

                            Point gauche,droite;
                            furthestLeftRight(result,gauche,droite);
                            Point measuredGauche = correctFE[gauche.y][gauche.x];
                            Point measuredDroite = correctFE[droite.y][droite.x];

                            Point measuredGauche2(measuredGauche.y,measuredGauche.x);
                            Point measuredDroite2(measuredDroite.y,measuredDroite.x);
                            circle(imgLines,gauche,30,Scalar(255,255,0),2);
                            circle(imgLines,droite,30,Scalar(255,255,0),2);

                            circle(imgLines,measuredGauche2,30,Scalar(0,255,255),2);
                            circle(imgLines,measuredDroite2,30,Scalar(0,255,255),2);

                            Point direction;
                            if((gauche-center).dot(gauche-center) < (droite-center).dot(droite-center)){
                                direction=gauche-center;
                            }
                            else
                                direction=droite-center;

                            if((point1-center).dot(point1-center)*0.1 < (point2-center).dot(point2-center)){
                                direction=Point(-1,-1)-center;//Pour renvoyer -1,-1
                            }
                            else if(measuredGauche2.x<20 || measuredGauche2.x>imgOriginal.size().width-20
                                    || measuredGauche2.y<20 || measuredGauche2.y>imgOriginal.size().height-20
                                    || measuredDroite2.x<20 || measuredDroite2.x>imgOriginal.size().width-20
                                    || measuredDroite2.y<20 || measuredDroite2.y>imgOriginal.size().height-20){
                                direction=Point(-1,-1)-center;//Pour renvoyer -1,-1
                            }
                            else{
                                line(imgLines, center, center+direction,Scalar(0,0,255),2);
                                circle(imgLines,center+direction,20,Scalar(0,0,255),2);
                                newArrows.push_back(Arrow(center,sqrt((point1-center).dot(point1-center)),direction ,center));
                            }

                        }
                        else if(objectColor[ARROW].algo==0){
                            if (isArrow(polygons,tolereratedError,imgLines,contours[i],newArrows)){
                                color=Scalar(0,255,255);
                                Point milieu=Point(0,0);
                            }
                        }

                    }
                    else if(shape==SQUARE && polygons.size()>=4){
                        color=Scalar(0,0,255);

                        if(objectColor[SQUARE].algo==1){
                            Point center,point1,point2;
                            PCAShape(polygons,center,point1,point2);
                            circle(imgLines, center, 3, Scalar(255, 0, 255), 2);
                            line(imgLines, center, point1, Scalar(0, 255, 0), 2);
                            line(imgLines, center, point2, Scalar(255, 255, 0), 2);

                            Mat imgContours = Mat::zeros( imgOriginal.size(), CV_8UC1 );
                            Mat imgAxis = Mat::zeros( imgOriginal.size(), CV_8UC1);

                            line(imgAxis, center, point1+10*(point1-center), Scalar(255), 2);
                            line(imgAxis, center, point1-10*(point1-center), Scalar(255), 2);

                            int lengthPoint1=(point1-center).dot(point1-center);

                            if(abs(lengthPoint1-16*(point2-center).dot(point2-center))>((double)tolerance/100.)*lengthPoint1){// lengthPoint1=(coté1/coté2)^4*length2
                                //Pour renvoyer -1,-1
                            }
                            else{
                                newSquares.push_back(Square(center));
                            }


                        }
                        else if(objectColor[SQUARE].algo==0){
                            if(isSquare(polygons,tolereratedError,imgLines,contours[i],newSquares)){
                                    color=Scalar(0,0,255);
                                    thickness=3;
                            }
                        }

                    }
                    else if(shape==CROSS && polygons.size()>=8){

                        if(objectColor[CROSS].algo==1){
                            Point center,point1,point2;
                            PCAShape(polygons,center,point1,point2);
                            circle(imgLines, center, 3, Scalar(255, 0, 255), 2);
                            line(imgLines, center, point1, Scalar(0, 255, 0), 2);
                            line(imgLines, center, point2, Scalar(255, 255, 0), 2);

                            Mat imgContours = Mat::zeros( imgOriginal.size(), CV_8UC1 );
                            Mat imgAxis = Mat::zeros( imgOriginal.size(), CV_8UC1);

                            line(imgAxis, center, point1+10*(point1-center), Scalar(255), 2);
                            line(imgAxis, center, point1-10*(point1-center), Scalar(255), 2);

                            int lengthPoint1=(point1-center).dot(point1-center);

                            if(abs(lengthPoint1-(point2-center).dot(point2-center))>((double)tolerance/100.)*lengthPoint1){// lengthPoint1=(coté1/coté2)^4*length2
                                //Pour renvoyer -1,-1
                            }
                            else{
                                newCrosses.push_back(Cross(center));
                            }

                        }
                        else if(objectColor[CROSS].algo==0){

                            if (isCross(polygons,tolereratedError,imgLines,contours[i],newCrosses)){
                                color=Scalar(0,0,255);
                                thickness=2;
                            }
                        }

                    }
                    if(activeObject==shape){
                        for(int j=0;j<polygons.size()-1;j++){
                            line(imgLines, polygons[j],polygons[j+1], color, thickness);
                        }
                    }

                    //line(imgLines, polygons[polygons.size()-1],polygons[0], color,thickness);

                }
            }

            if(activeObject==shape){
                imshow("Thresholded Image", imgThresholded); //show the thresholded image
            }
        }

        int vitesseMax=2;
        for(int i=shapes[SQUARE].size()-1; i>=0;i--){
                shapes[SQUARE][i].addLifeTime(deltaTms);
                shapes[SQUARE][i].addLastTime(deltaTms);

                if(newSquares.size()>0){
                    Point dist = newSquares[0].getPos()-shapes[SQUARE][i].getPos();
                    int jmin=0;
                    int distMin=sqrt(dist.dot(dist));
                    int distTemp=0;
                    for(int j=1;j<newSquares.size();j++){
                        dist = newSquares[j].getPos()-shapes[SQUARE][i].getPos();
                        distTemp=sqrt(dist.dot(dist));
                        if(distTemp<distMin){
                            distMin=distTemp;
                            jmin=j;
                        }
                    }

                    if(distMin<(vitesseMax*deltaTms)){
                        shapes[SQUARE][i].setPos(newSquares[jmin].getPos());
                        shapes[SQUARE][i].addSeenTime(deltaTms);
                        shapes[SQUARE][i].setLastTime(0);
                        newSquares.erase(newSquares.begin()+jmin);
                    }


                }



                if(shapes[SQUARE][i].getLastTime()>500){
                        shapes[SQUARE].erase(shapes[SQUARE].begin()+i);
                }
                else if(shapes[SQUARE][i].getSeenTime()>100){
                        circle(imgLines,shapes[SQUARE][i].getPos(),vitesseMax*deltaTms,Scalar(0,0,255),2);
                        ostringstream sstream3;
                        sstream3 << i;
                        string varAsString3 = sstream3.str();
                        putText(imgLines,"Square "+varAsString3,shapes[SQUARE][i].getPos(),FONT_HERSHEY_PLAIN,3,Scalar(0,0,255));
                }
        }
        while (newSquares.size()>0){
            shapes[SQUARE].push_back(newSquares.back());
            newSquares.pop_back();
            shapes[SQUARE].back().addSeenTime(deltaTms);
        }

        for(int i=shapes[CROSS].size()-1; i>=0;i--){
                shapes[CROSS][i].addLifeTime(deltaTms);
                shapes[CROSS][i].addLastTime(deltaTms);

                if(newCrosses.size()>0){
                    Point dist = newCrosses[0].getPos()-shapes[CROSS][i].getPos();
                    int jmin=0;
                    int distMin=sqrt(dist.dot(dist));
                    int distTemp=0;
                    for(int j=1;j<newCrosses.size();j++){
                        dist = newCrosses[j].getPos()-shapes[CROSS][i].getPos();
                        distTemp=sqrt(dist.dot(dist));
                        if(distTemp<distMin){
                            distMin=distTemp;
                            jmin=j;
                        }
                    }

                    if(distMin<(vitesseMax*deltaTms)){
                        shapes[CROSS][i].setPos(newCrosses[jmin].getPos());
                        shapes[CROSS][i].addSeenTime(deltaTms);
                        shapes[CROSS][i].setLastTime(0);
                        newCrosses.erase(newCrosses.begin()+jmin);
                    }


                }



                if(shapes[CROSS][i].getLastTime()>500){
                        shapes[CROSS].erase(shapes[CROSS].begin()+i);
                }
                else if(shapes[CROSS][i].getSeenTime()>100){
                        circle(imgLines,shapes[CROSS][i].getPos(),vitesseMax*deltaTms,Scalar(0,0,255),2);
                        ostringstream sstream3;
                        sstream3 << i;
                        string varAsString3 = sstream3.str();
                        putText(imgLines,"Cross "+varAsString3,shapes[CROSS][i].getPos(),FONT_HERSHEY_PLAIN,3,Scalar(0,0,255));
                }
        }
        while (newCrosses.size()>0){
            shapes[CROSS].push_back(newCrosses.back());
            newCrosses.pop_back();
            shapes[CROSS].back().addSeenTime(deltaTms);
        }
        //cout << "nbr arrows comfirmée : "<<shapes[ARROW].size()<<endl;
        //cout << "nbr arrows detect : "<<newArrows.size()<<endl;
        for(int i=shapes[ARROW].size()-1; i>=0;i--){
                shapes[ARROW][i].addLifeTime(deltaTms);
                shapes[ARROW][i].addLastTime(deltaTms);

                if(newArrows.size()>0){
                    Point dist = newArrows[0].getPos()-shapes[ARROW][i].getPos();
                    int jmin=0;
                    int distMin=sqrt(dist.dot(dist));
                    int distTemp=0;
                    for(int j=1;j<newArrows.size();j++){
                        dist = newArrows[j].getPos()-shapes[ARROW][i].getPos();
                        distTemp=sqrt(dist.dot(dist));
                        if(distTemp<distMin){
                            distMin=distTemp;
                            jmin=j;
                        }
                    }

                    if(distMin<(vitesseMax*deltaTms)){
                        shapes[ARROW][i].setPos(newArrows[jmin].getPos());
                        shapes[ARROW][i].setDirection(newArrows[jmin].getDirection());
                        shapes[ARROW][i].addSeenTime(deltaTms);
                        shapes[ARROW][i].setLastTime(0);
                        newArrows.erase(newArrows.begin()+jmin);
                    }


                }



                if(shapes[ARROW][i].getLastTime()>500){
                        shapes[ARROW].erase(shapes[ARROW].begin()+i);
                }
                else if(shapes[ARROW][i].getSeenTime()>100){
                        circle(imgLines,shapes[ARROW][i].getPos(),vitesseMax*deltaTms,Scalar(0,0,255),2);
                        ostringstream sstream3;
                        sstream3 << i;
                        string varAsString3 = sstream3.str();
                        putText(imgLines,"Arrow "+varAsString3,shapes[ARROW][i].getPos(),FONT_HERSHEY_PLAIN,3,Scalar(0,0,255));
                }
        }
        while (newArrows.size()>0){
            shapes[ARROW].push_back(newArrows.back());
            newArrows.pop_back();
            shapes[ARROW].back().addSeenTime(deltaTms);
        }

        if(activeObject==GRASS){
            imshow("Thresholded Image", imgMaskGrass); //show the thresholded image
        }

        if(alphaI!=alphaIOld){
            correctFE=vector<vector<Point> >(imgOriginal.size().height,vector<Point>(imgOriginal.size().width,Point(0,0)));
            computeFECorrection2(imgOriginal.size(),(double) alphaI/1000*1.5,correctFE);
            correctToData(correctFE,dataCorrectFE,imgOriginal.channels(),imgOriginal.cols);
            alphaIOld=alphaI;
        }

        currentTime=(double) clock()/CLOCKS_PER_SEC;

        //cout<< time <<" "<<previousTime << endl;
        ostringstream sstream;
        deltaT=currentTime-previousTime;
        deltaTms=1000*deltaT;
        double fpsMeasure=(double) 1./(deltaT);
        sstream << fpsMeasure;
        string varAsString = sstream.str();
        Scalar fpsColor=Scalar(0,255,0);



        zmq::message_t request;
        socket.recv(&request,ZMQ_DONTWAIT);

        while(request.size()!=0){
            string msg = string(static_cast<char*>(request.data()), request.size());
            if (msg!="REQ")
            {
                cout << "Strange answer from server : " << msg << endl;
            }

            string ans;
            ostringstream streamAns;

            if(shapes[SQUARE].size()>0){

                Point pos=shapes[SQUARE][0].getPos();
                streamAns << "PAD ";
                streamAns << pos.x <<" ";
                streamAns << pos.y;
                streamAns << " -1 -1 ";
            }
            else{
                streamAns << "PAD -1 -1 -1 -1 ";
            }
            if(shapes[ARROW].size()>0){
                    Point pos=shapes[ARROW][0].getPos();
                    Point dir=shapes[ARROW][0].getDirection();//((double)11./18)*
                    int maxIndex=0;
                    int maxLength=dir.dot(dir);

                    for(int i=1;i<shapes[ARROW].size();i++){
                        Point tempPos=shapes[ARROW][i].getPos();
                        Point tempDir=shapes[ARROW][i].getDirection();
                        int tempLength=tempDir.dot(tempDir);
                        if(tempPos.x + tempDir.x != -1 && tempLength>maxLength){
                            maxIndex=i;
                            maxLength=tempLength;
                        }
                    }
                    pos=shapes[ARROW][maxIndex].getPos();
                    dir=shapes[ARROW][maxIndex].getDirection();

                    streamAns << "ARROW ";
                    streamAns << pos.x <<" ";
                    streamAns << pos.y<<" ";
                    streamAns << pos.x + dir.x<<" ";
                    streamAns << pos.y + dir.y << " ";
            }
            else{
                streamAns << "ARROW -1 -1 -1 -1 ";
            }
            if(shapes[CROSS].size()>0){
                Point pos=shapes[CROSS][0].getPos();
                streamAns << "CROSS ";
                streamAns << pos.x <<" ";
                streamAns << pos.y;
                streamAns << " -1 -1";
            }
            else{
                streamAns << "CROSS -1 -1 -1 -1";
            }

            ans = streamAns.str();
            zmq::message_t answer (ans.size());
            memcpy ((void *) answer.data (), ans.c_str(), ans.size());

            socket.send (answer);

            //ZMQ
            //Next request from client
            socket.recv(&request,ZMQ_DONTWAIT);

        }



        //std::cout << "Received Hello" << std::endl


        if((double) 1./(deltaT)<FPS){
            fpsColor=Scalar(0,0,255);
        }
        putText(imgLines,"fps : "+varAsString,Point(0,12),FONT_HERSHEY_PLAIN,1,fpsColor);
        if(isRecording){
            putText(imgLines,"recording",Point(frameSize.width-80,12),FONT_HERSHEY_PLAIN,1,Scalar(0,0,255));
        }
        if(paused){
            putText(imgLines,"pause",Point(frameSize.width-80,frameSize.height-12),FONT_HERSHEY_PLAIN,1,Scalar(0,0,255));
        }

        Mat mask,mask2;
        vector<Mat> channels;

        inRange(imgLines, Scalar(0, 0, 1), Scalar(255, 255, 255), mask);
        inRange(imgLines, Scalar(0, 1, 0), Scalar(255, 255, 255), mask2);
        mask+=mask2;
        inRange(imgLines, Scalar(0, 1, 0), Scalar(255, 255, 255), mask2);
        mask+=mask2;

        channels.push_back(mask);
        channels.push_back(mask);
        channels.push_back(mask);
        merge(channels, mask2);
        imgScr=(imgOriginal-mask2)+imgLines;

        imshow("No fish Eye", (imgNoFE-mask2)+imgLines); //show the original image
        imshow("Original", imgScr);

        int key=-1;
        if (deltaT> (double) 1/FPS){
            key=waitKey(1);
        }
        else{
            //cout << (int)round(((double)1/FPS-time+previousTime)*1000) <<endl;
            key=waitKey(1+(int) round(((double)1/FPS-deltaT)*1000));
            //key=waitKey(1);

        }
        previousTime=currentTime;

        //cout << key << endl;

        if ( key== 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
        {
            cout << "esc key is pressed by user" << endl;
            break;
        }
        else if (key==114)// touche R
        {
            imgLines = Mat::zeros( imgOriginal.size(), CV_8UC3 );
        }
        else if (key==112)// touche p
        {
            paused = not paused;
        }
        else if (key==113 && file)// touche q
        {
            cap.set(CV_CAP_PROP_POS_MSEC,cap.get(CV_CAP_PROP_POS_MSEC)-1000);
        }
        else if (key==100 && file)// touche d
        {
            cap.set(CV_CAP_PROP_POS_MSEC,cap.get(CV_CAP_PROP_POS_MSEC)+1000);
        }
        else if(key==118)//touche V
        {
            if(isRecording){
                oVideoWriter.release();
                isRecording=false;
                cout << "fin"<<endl;
            }
            else{
                ostringstream sstream2;
                sstream2 << std::time(NULL);std::
                string varAsString2 = sstream2.str();
                oVideoWriter=VideoWriter("HexaTeam"+varAsString2+".avi", -1, 30, frameSize, true);
                if ( !oVideoWriter.isOpened() ) //if not initialize the VideoWriter successfully, exit the program
               {
                    cout << "ERROR: Failed to write the video " << "HexaTeam"+varAsString2+".avi" << endl;
                    return -1;
               }
                cout << "debut"<<endl;
               isRecording=true;
            }

        }
    }

    ofstream colorFileO("objectColor.txt");

    if(colorFileO){
        for(int i=0;i<4;i++){
            colorFileO << objectColor[i].lowH << ";";
            colorFileO << objectColor[i].lowS << ";";
            colorFileO << objectColor[i].lowV << ";";
            colorFileO << objectColor[i].highH << ";";
            colorFileO << objectColor[i].highS << ";";
            colorFileO << objectColor[i].highV << ";";
            colorFileO << objectColor[i].algo << ";";
        }
    }
    else
        cout << "Impossible de sauvegarder les couleurs."<<endl;

    delete[] dataCorrectFE;

    return 0;
}

bool isArrow(const vector<Point>& polygons, float epsilon, Mat& img,const vector<Point>& contour, vector<Arrow> &arrows){
    vector<Point> side = vector<Point>(polygons.size(),Point());
    vector<float> lengthSquare=vector<float>(polygons.size(),0);
    Point direction;
    for(int i=0;i<side.size()-1;i++){
        side[i]=polygons[i+1]-polygons[i];
        lengthSquare[i]=(float)side[i].x*(float)side[i].x+(float)side[i].y*(float)side[i].y;
    }
    side[side.size()-1]=polygons[0]-polygons[side.size()-1];
    lengthSquare[side.size()-1]=(float)side[side.size()-1].x*(float)side[side.size()-1].x+(float)side[side.size()-1].y*(float)side[side.size()-1].y;

    int iMax=0;
    float maxLength=lengthSquare[0];
    for(int i=1;i<lengthSquare.size();i++){
        if(lengthSquare[i]>maxLength){
            maxLength=lengthSquare[i];
            iMax=i;
        }
    }

    int iMax2=0;
    float maxLength2=lengthSquare[0];
    if(iMax==0){
        iMax2=1;
        maxLength2=lengthSquare[1];
    }

    for(int i=0;i<lengthSquare.size();i++){
        if(lengthSquare[i]>maxLength2 && i!=iMax){
            maxLength2=lengthSquare[i];
            iMax2=i;
        }
    }

    for(int j=0;j<polygons.size()-1;j++){
        line(img, polygons[j],polygons[j+1], Scalar(0,255,0), 1);
    }

    if(abs(lengthSquare[iMax]-lengthSquare[iMax2])>epsilon*lengthSquare[iMax]){
        //cout << "longueur des longs cotés pas bonne"<<endl;
        return false;
    }


    line(img, polygons[iMax],polygons[iMax]+side[iMax],Scalar(255,0,255),2);
    line(img, polygons[iMax2],polygons[iMax2]+side[iMax2], Scalar(255,0,255),2);
    putText(img,"Side1",polygons[iMax]+0.*side[iMax],FONT_HERSHEY_PLAIN,1,Scalar(0,0,255));
    putText(img,"Side2",polygons[iMax2]+0.*side[iMax2],FONT_HERSHEY_PLAIN,1,Scalar(0,0,255));

    float ps=abs((float)side[iMax].x*(float)side[iMax2].x + (float)side[iMax].y*(float)side[iMax2].y);
    //cout << (float) ps*ps << endl;
    //cout << (float) lengthSquare[iMax2]* (float) lengthSquare[iMax]<< endl;
    if( ps*ps*(1+0.1) < lengthSquare[iMax]*lengthSquare[iMax2]){
        //cout << "alignement pas bon"<< polygons[iMax].x<<" "<< polygons[iMax].y <<endl;
        return false;
    }


    Point smallSide1=polygons[iMax2]-(polygons[iMax]+side[iMax]);
    Point smallSide2=polygons[iMax]-(polygons[iMax2]+side[iMax2]);


    float lengthSSide1 = (float)smallSide1.x*(float)smallSide1.x + (float)smallSide1.y*(float)smallSide1.y;
    float lengthSSide2 = (float)smallSide2.x*(float)smallSide2.x + (float)smallSide2.y*(float)smallSide2.y;

    //cout << lengthSSide1 << "&" << lengthSSide2 << "compared to "<< lengthSquare[iMax]<< " " << polygons[iMax].x<<" "<< polygons[iMax].y <<endl;
    //Attention ! On ne mesure pas toute la longueur de la flèche mais bien de l'arrière au creux de la hanse
    if (lengthSSide1>0.0167*lengthSquare[iMax] || lengthSSide2>0.0167*lengthSquare[iMax]){
        //cout << "petits cotes trop grands"<< polygons[iMax].x<<" "<< polygons[iMax].y <<endl;
        return false;
    }


    direction=0.5*(side[iMax]-side[iMax2]);//les cotes sont opposés...
    Point milieuFleche;
    for(int i=0;i<polygons.size();i++){
        milieuFleche=milieuFleche+polygons[i];
    }
    milieuFleche=milieuFleche*(1./polygons.size());
    Point milieuRect=0.25*(polygons[iMax]+polygons[iMax]+side[iMax]+polygons[iMax2]+polygons[iMax2]+side[iMax2]);
    if(direction.dot(milieuFleche-milieuRect)<0){
        direction=-direction;
    }
    vector<Point> poly2;
    poly2.push_back(polygons[iMax]);
    poly2.push_back(polygons[iMax]+side[iMax]);
    poly2.push_back(polygons[iMax2]);
    poly2.push_back(polygons[iMax2]+side[iMax2]);

    int areaPoly=moments(poly2).m00;
    int areaContour=contourArea(contour);//rapport théorique 0.628

    //cout << areaPoly << "&" << areaContour << endl;
    if((float)areaPoly>(float)0.65*areaContour)
    {
        //cout << "aire mal placée trop grande "<< polygons[iMax].x<<" "<< polygons[iMax].y <<endl;
        return false;
    }
    if((float)areaPoly<(float)0.50*areaContour)
    {
        //cout << "aire mal placée trop petite"<< polygons[iMax].x<<" "<< polygons[iMax].y <<endl;
        return false;
    }

    //condition sur le bout de la flèche : il doit tenir dans un carré pas trop grand

    //putText(img,"Arrow",Point(milieuRect),FONT_HERSHEY_PLAIN,1,Scalar(0,0,255));
    line(img, milieuRect,milieuRect+11*direction*((double)1./18), Scalar(255,255,0));
    circle(img,milieuRect+11*direction*((double)1./18), 20,Scalar(255,255,0));

    arrows.push_back(Arrow(milieuRect,sqrt(lengthSSide1),direction,milieuFleche));
    return true;
}

bool compareLengthIndex(const struct lengthIndex a, const struct lengthIndex b)
{
   return a.length>b.length;
}

bool compareLengthIndex2(const struct lengthIndex a, const struct lengthIndex b)
{
   return a.index<b.index;
}

bool isCross(const vector<Point>& polygons, float epsilon, Mat& img,const vector<Point>& contour, vector<Cross> &crosses){
    vector<Point> side = vector<Point>(polygons.size(),Point());
    vector<float> lengthSquare=vector<float>(polygons.size(),0);

    for(int i=0;i<side.size()-1;i++){
        side[i]=polygons[i+1]-polygons[i];
        lengthSquare[i]=(float)side[i].x*(float)side[i].x+(float)side[i].y*(float)side[i].y;
    }
    side[side.size()-1]=polygons[0]-polygons[side.size()-1];
    lengthSquare[side.size()-1]=(float)side[side.size()-1].x*(float)side[side.size()-1].x+(float)side[side.size()-1].y*(float)side[side.size()-1].y;

    vector<struct lengthIndex> lengthMax;
    for(int i=0; i<lengthSquare.size();i++){
        lengthIndex temp;
        temp.length=lengthSquare[i];
        temp.index=i;
        lengthMax.push_back(temp);
    }

    partial_sort(lengthMax.begin(),lengthMax.begin()+8, lengthMax.end(),compareLengthIndex);

    float averageLength;
    for(int i=0;i<8;i++){
        averageLength+=lengthMax[i].length/8;
    }

    bool res=true;

    /*for(int i=0;i<8;i++){
        line(img, polygons[lengthMax[i].index],polygons[lengthMax[i].index]+side[lengthMax[i].index],Scalar(255,0,0),3);
        circle(img,polygons[lengthMax[i].index], 20,Scalar(255,255,0));
        circle(img,polygons[lengthMax[i].index]+side[lengthMax[i].index], 20,Scalar(255,255,0));
    }*/

    for(int i=0;i<8;i++){
        //cout << lengthMax[i].length <<endl;
        if(abs(lengthMax[i].length-averageLength)>epsilon*averageLength){
            //cout << "longueurs pas toutes égales" << endl;
            return false;
        }
    }




    vector<struct lengthIndex> axeMax[2];
    axeMax[0].push_back(lengthMax[0]);
    for(int i=1;i<8;i++){
        float ps=abs((float)side[lengthMax[0].index].x*(float)side[lengthMax[i].index].x + (float)side[lengthMax[0].index].y*(float)side[lengthMax[i].index].y);

        if(abs(ps*ps-lengthMax[0].length*lengthMax[i].length)<epsilon*lengthMax[0].length*lengthMax[0].length){
            axeMax[0].push_back(lengthMax[i]);
        }
        else if(ps<epsilon*lengthMax[0].length){
            axeMax[1].push_back(lengthMax[i]);
        }
        else{
            //cout << "mauvaise orientation des cotes"<<endl;
            return false;
        }
    }
    if(axeMax[0].size()!=4 || axeMax[1].size()!=4){
        //cout << "répartition étrange des cotés"<<endl;
        return false;
    }

    sort(axeMax[0].begin(),axeMax[0].end(),compareLengthIndex2);//tri selon l'index donc selon l'ordre ds le contour
    sort(axeMax[1].begin(),axeMax[1].end(),compareLengthIndex2);

    int paires[4][2];
    for(int i=0;i<2;i++){
        float ps=(float)side[axeMax[i][0].index].x*(float)side[axeMax[i][1].index].x + (float)side[axeMax[i][0].index].y*(float)side[axeMax[i][1].index].y;
        if (ps>0){
            paires[2*i][0]=axeMax[i][0].index;
            paires[2*i][1]=axeMax[i][3].index;
            paires[2*i+1][0]=axeMax[i][1].index;
            paires[2*i+1][1]=axeMax[i][2].index;
        }
        else{
            paires[2*i][0]=axeMax[i][0].index;
            paires[2*i][1]=axeMax[i][1].index;
            paires[2*i+1][0]=axeMax[i][2].index;
            paires[2*i+1][1]=axeMax[i][3].index;
        }
    }

    for(int i=0;i<4;i++){
        Point smallSide=polygons[paires[i][1]]-(polygons[paires[i][0]]+side[paires[i][0]]);
        float length = (float)smallSide.x*(float)smallSide.x + (float)smallSide.y*(float)smallSide.y;
        if(abs(length-0.01*lengthSquare[paires[i][0]])<epsilon*lengthSquare[paires[i][0]]){
            smallSide=polygons[paires[i][0]]-(polygons[paires[i][1]]+side[paires[i][1]]);
            length = (float)smallSide.x*(float)smallSide.x + (float)smallSide.y*(float)smallSide.y;
            if(abs(length-0.01*lengthSquare[paires[i][0]])>epsilon*lengthSquare[paires[i][0]]){
                //cout << "epaisseur trop grande"<< endl;
                return false;
            }
        }
        else {
            return false;
        }
    }

    vector<Point> poly2;
    for(int i=0;i<8;i++){
        poly2.push_back(polygons[lengthMax[i].index]);
        poly2.push_back(polygons[(lengthMax[i].index+1)%polygons.size()]);
    }

    int areaPoly=moments(poly2).m00;
    int areaContour=contourArea(contour);//rapport théorique 0.628

    if((float)areaPoly < (float)0.60*areaContour)
    {
        //cout << "aire mal placée "<<endl;
        return false;
    }

    Point milieu;
    for(int i=0;i<8;i++){
        milieu=milieu+polygons[lengthMax[i].index]+polygons[(lengthMax[i].index+1)%polygons.size()];
        //circle(img,polygons[lengthMax[i].index], 10,Scalar(0,255,255),2);
        //circle(img,polygons[(lengthMax[i].index+1)%polygons.size()], 10,Scalar(0,255,255),2);
    }
    milieu=milieu*(1./16);
    circle(img,milieu, 30,Scalar(0,0,255),3);
    putText(img,"Cross",Point(milieu),FONT_HERSHEY_PLAIN,1,Scalar(0,0,255));
    crosses.push_back(Cross(milieu,sqrt(lengthMax[0].length)));
    return true;
}

bool isSquare(const vector<Point>& polygons, float epsilon,Mat& img, const vector<Point>& contour, vector<Square> &squares){
    vector<Point> side = vector<Point>(polygons.size(),Point());
    vector<float> lengthSquare=vector<float>(polygons.size(),0);

    for(int i=0;i<side.size()-1;i++){
        side[i]=polygons[i+1]-polygons[i];
        lengthSquare[i]=(float)side[i].x*(float)side[i].x+(float)side[i].y*(float)side[i].y;
    }
    side[side.size()-1]=polygons[0]-polygons[side.size()-1];
    lengthSquare[side.size()-1]=(float)side[side.size()-1].x*(float)side[side.size()-1].x+(float)side[side.size()-1].y*(float)side[side.size()-1].y;

    int iMax=0;
    float maxLength=lengthSquare[0];
    for(int i=1;i<lengthSquare.size();i++){
        if(lengthSquare[i]>maxLength){
            maxLength=lengthSquare[i];
            iMax=i;
        }
    }

    int iMax2=0;
    float maxLength2=lengthSquare[0];
    if(iMax==0){
        iMax2=1;
        maxLength2=lengthSquare[1];
    }

    for(int i=0;i<lengthSquare.size();i++){
        if(lengthSquare[i]>maxLength2 && i!=iMax){
            maxLength2=lengthSquare[i];
            iMax2=i;
        }
    }

    if(abs(lengthSquare[iMax]-lengthSquare[iMax2])>epsilon*lengthSquare[iMax]){
        //cout << " Square : longueur des longs cotés pas bonne"<<endl;
        return false;
    }

    float ps=abs((float)side[iMax].x*(float)side[iMax2].x + (float)side[iMax].y*(float)side[iMax2].y);
    if( ps*ps*(1+0.1) < lengthSquare[iMax]*lengthSquare[iMax2]){
        //cout << "Square : alignement pas bon"<<endl;
        return false;
    }

    Point smallSide1=polygons[iMax2]-(polygons[iMax]+side[iMax]);
    Point smallSide2=polygons[iMax]-(polygons[iMax2]+side[iMax2]);

    float lengthSSide1 = (float)smallSide1.x*(float)smallSide1.x + (float)smallSide1.y*(float)smallSide1.y;
    float lengthSSide2 = (float)smallSide2.x*(float)smallSide2.x + (float)smallSide2.y*(float)smallSide2.y;

    if (abs(lengthSSide1-0.5*lengthSquare[iMax])>epsilon*lengthSSide1 || abs(lengthSSide2-0.5*lengthSquare[iMax])>epsilon*lengthSSide1){//0.25
        //cout << "petits cotes pas de la bonne taille"<<endl;
        return false;
    }

    ps=abs((float)side[iMax].x*(float)smallSide1.x + (float)side[iMax].y*(float)smallSide1.y);

    if (abs(ps)>epsilon*lengthSSide1){
        //cout << "Mauvais angles droits"<<endl;
        return false;
    }

    vector<Point> poly2;
    poly2.push_back(polygons[iMax]);
    poly2.push_back(polygons[iMax]+side[iMax]);
    poly2.push_back(polygons[iMax2]);
    poly2.push_back(polygons[iMax2]+side[iMax2]);

    int areaPoly=moments(poly2).m00;
    int areaContour=contourArea(contour);//rapport théorique 0.99


    if((float)abs(areaPoly-(float)0.99*areaContour)>0.05*(float)areaContour)
    {
        //cout << "aire mal placée "<<endl;
        return false;
    }

    line(img, polygons[iMax],polygons[iMax2],Scalar(255,0,0),2);
    line(img, polygons[iMax]+side[iMax],polygons[iMax2]+side[iMax2],Scalar(255,0,0),2);
    Point milieu=0.25*(2*polygons[iMax]+side[iMax]+2*polygons[iMax2]+side[iMax2]);
    putText(img,"Rectangle",Point(milieu),FONT_HERSHEY_PLAIN,1,Scalar(0,0,255));

    //cout << "OK !"<<endl;
    squares.push_back(Square(milieu, sqrt(lengthSSide1)));
    return true;
}


void computeFECorrection(Size siz, double alpha, vector<vector<Point> >& correct){
    int nbX=siz.height;
    int nbY=siz.width;
    double tanAlpha=tan(alpha);
    double xd,yd;
    int xi,yi;
    double dist,frac,tan2,rx,ry;
    //cout << alpha << " & "<<tanAlpha << endl;
    for(int i=0;i<nbX;i++){
        for(int j=0; j<nbY;j++){
            rx=(double) (i-nbX/2)*2/nbY;
            ry=(double) 2*j/nbY-1;
            dist=sqrt(rx*rx+ry*ry);
            tan2=tan(alpha*dist)/tanAlpha;
            frac=rx/(dist);//alpha*dist
            yd=nbY/2*tan2*sqrt(1-frac*frac)*sgn(ry);
            xd=nbY/2*tan2*frac;
            xi=floor((double) nbX/2+xd+0.5);
            yi=floor((double) nbY/2+yd+0.5);

            if(i==nbX-1 && j==nbY-1){
                //cout << xi << " & "<< yi <<" & " << nbX << " & "<< nbY << " & "<< dist << endl;
            }
            if(xi < nbX && yi< nbY && xi>=0 && yi>=0){
                correct[i][j]=Point(xi,yi);
            }
            //else
                //cout << "trop grand ou trop petit !"<<endl;

        }
    }

}

void computeFECorrection2(Size siz, double alpha, vector<vector<Point> >& correct){
    int nbX=siz.height;
    int nbY=siz.width;
    double tanAlpha=tan(alpha);
    double xd,yd;
    int xi,yi;
    double dist,frac,tan2,rx,ry,phi;
    //cout << alpha << " & "<<tanAlpha << endl;
    for(int i=0;i<nbX;i++){
        for(int j=0; j<nbY;j++){

            rx= (double) i-nbX/2;
            ry= (double) j-nbY/2;

            dist=sqrt(rx*rx+ry*ry);
            tan2=tanAlpha*dist/((double)nbY/2);
            phi=atan(tan2);

            yd=nbY/2*phi*ry/dist/alpha;
            xd=nbY/2*phi*rx/dist/alpha;

            xi=floor((double) nbX/2+xd+0.5);
            yi=floor((double) nbY/2+yd+0.5);

            if(i==nbX-1 && j==nbY-1){
                //cout << xi << " & "<< yi <<" & " << nbX << " & "<< nbY << " & "<< dist << endl;
            }
            if(xi < nbX && yi< nbY && xi>=0 && yi>=0){
                correct[i][j]=Point(xi,yi);
            }

            //else
                //cout << "trop grand ou trop petit !"<<endl;

        }
    }

}

void correctFishEye(Mat& src, Mat& dst, const vector<vector<Point> >& correct){//PRENDRE ORIGINE AU MILIEU !
    dst=Mat::zeros( src.size(), CV_8UC3 );
    int chan=src.channels();
    int col=src.cols;
    int chanCol=chan*col;

    //cout << alpha << " & "<<tanAlpha << endl;
    int index=0;
    int index2=0;
    int index3=0;

    for(int i=0;i<src.size().height;i++){
        for(int j=0; j<src.size().width;j++){
            index3=chan*(col*correct[i][j].x+correct[i][j].y);
            for(int k=0;k<3;k++){
                //dst.data[chan*(col*correct[i][j].x+correct[i][j].y)+k]=src.data[chan*(col*i+j)+k];
                //dst.data[dst.channels()*(dst.cols*i+j)+k]=255;
                dst.data[index3]=src.data[index];
                index++;
                index3++;
            }
        }
    }
}

void correctFishEye2(Mat& src, Mat& dst, const vector<vector<Point> >& correct){//PRENDRE ORIGINE AU MILIEU !
    dst=Mat::zeros( src.size(), CV_8UC3 );
    int chan=src.channels();
    int col=src.cols;
    int chanCol=chan*col;

    //cout << alpha << " & "<<tanAlpha << endl;
    int index=0;
    int index2=0;
    int index3=0;

    for(int i=0;i<src.size().height;i++){
        for(int j=0; j<src.size().width;j++){
            index3=chan*(col*correct[i][j].x+correct[i][j].y);
            for(int k=0;k<3;k++){
                //dst.data[chan*(col*correct[i][j].x+correct[i][j].y)+k]=src.data[chan*(col*i+j)+k];
                //dst.data[dst.channels()*(dst.cols*i+j)+k]=255;
                dst.data[index]=src.data[index3];
                index++;
                index3++;
            }
        }
    }
}

void correctFishEye3(Mat& src, Mat& dst, unsigned int data[]){
    dst=Mat::zeros( src.size(), CV_8UC3 );
    int sizeData=src.channels()*(src.cols*(src.size().height-1)+(src.size().width-1))+3;
    uchar* ptrDst=dst.data;
    unsigned int* ptrData=data;

    for(int i=0;i<sizeData;i++){
        *ptrDst=src.data[*ptrData];
        //dst.data[i]=src.data[data[i]];
        ptrDst++;
        ptrData++;
    }
}

void correctToData(const vector<vector<Point> >& correct, unsigned int data[], int chan, int col){
    for(int i=0;i<correct.size();i++){
        for(int j=0; j<correct[0].size();j++){
            for(int k=0;k<3;k++){
                data[chan*(col*i+j)+k]=chan*(col*correct[i][j].x+correct[i][j].y)+k;
            }
        }
    }
}

void furthestLeftRight(Mat& src, Point& pointLeft, Point& pointRight){
    Point maxi(0,0);
    Point mini(src.size().width,src.size().height);


    for(int i=0;i<src.size().height;i++){
        for(int j=0; j<src.size().width;j++){
            for(int k=0;k<1;k++){
                if(src.data[src.channels()*(src.cols*i+j)+k]!=0){
                    if(j<mini.x){
                        mini=Point(j,i);
                    }
                    if(j>maxi.x){
                        maxi=Point(j,i);
                    }
                }
            }
        }
    }
    pointLeft=mini;
    pointRight=maxi;
}


void PCAShape(vector<Point> polygons, Point& center, Point& point1, Point& point2 ){
    //Construct a buffer used by the pca analysis
    int sz = static_cast<int>(polygons.size());
    Mat data_pts = Mat(sz, 2, CV_64FC1);
    for (int i = 0; i < data_pts.rows; ++i)
    {
        data_pts.at<double>(i, 0) = polygons[i].x;
        data_pts.at<double>(i, 1) = polygons[i].y;
    }
    //Perform PCA analysis
    PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);
    //Store the center of the object
    Point cntr = Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)),
                        static_cast<int>(pca_analysis.mean.at<double>(0, 1)));
    //Store the eigenvalues and eigenvectors
    vector<Point2d> eigen_vecs(2);
    vector<double> eigen_val(2);
    for (int i = 0; i < 2; ++i)
    {
        eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                                pca_analysis.eigenvectors.at<double>(i, 1));
        eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
    }
    // Draw the principal components
    Point p1 = cntr + 0.02 * Point(static_cast<int>(eigen_vecs[0].x * eigen_val[0]), static_cast<int>(eigen_vecs[0].y * eigen_val[0]));
    Point p2 = cntr - 0.02 * Point(static_cast<int>(eigen_vecs[1].x * eigen_val[1]), static_cast<int>(eigen_vecs[1].y * eigen_val[1]));
    center = cntr;
    point1 = p1;
    point2 = p2;
}


static void objectCallback(int index, void* userdata)
{
    activeObject=index;
    setTrackbarPos("LowH","Control",objectColor[index].lowH);
    setTrackbarPos("HighH","Control",objectColor[index].highH);
    setTrackbarPos("LowS","Control",objectColor[index].lowS);
    setTrackbarPos("HighS","Control",objectColor[index].highS);
    setTrackbarPos("LowV","Control",objectColor[index].lowV);
    setTrackbarPos("HighV","Control",objectColor[index].highV);
    setTrackbarPos("Deterministe / Statistique","Control",objectColor[index].algo);

}
static void lowHCallback(int level, void* userdata)
{
    objectColor[getTrackbarPos("Object","Control")].lowH=level;
}
static void highHCallback(int level, void* userdata)
{
    objectColor[getTrackbarPos("Object","Control")].highH=level;
}
static void lowSCallback(int level, void* userdata)
{
    objectColor[getTrackbarPos("Object","Control")].lowS=level;
}
static void highSCallback(int level, void* userdata)
{
    objectColor[getTrackbarPos("Object","Control")].highS=level;
}
static void lowVCallback(int level, void* userdata)
{
    objectColor[getTrackbarPos("Object","Control")].lowV=level;
}
static void highVCallback(int level, void* userdata)
{
    objectColor[getTrackbarPos("Object","Control")].highV=level;
}

static void algoCallback(int level, void* userdata)
{
    objectColor[getTrackbarPos("Object","Control")].algo=level;
}

