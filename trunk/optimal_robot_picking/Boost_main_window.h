/*
 *  The main window class. 
 *
 *  Created on: Jan 30, 2015
 *  Author: Jingjin Yu
 */

#ifndef _MAIN_WINDOW_H_
#define _MAIN_WINDOW_H_

#include <QMainWindow>
#include <QMenuBar>
#include <QToolBar>
#include <QAction>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QLineEdit>
#include <QGraphicsEllipseItem>
#include <QTimer>
#include <QApplication>
#include <QLabel>
//#include <unistd.h>
//#include <chrono>
#include <windows.h>
#include <time.h>
#include <random>
#include "Boost_helper_functions.h"
#include "Boost_types.h"
#include "Boost_graphics_item.h"
#include "Boost_roadmap.h"
#include "GraphicsViewNavigation.h"


class MainWindow : public QMainWindow 
{
    Q_OBJECT

public:
	enum Option { NoOption = 0x0,
		Size     = 0x1,
		Position = 0x2,
		State    = 0x4};
	QString m_fileNameq;
	std::string m_fileName;
	Q_DECLARE_FLAGS(Options, Option)
	
    MainWindow( QApplication *pApp, QString& fileFolder, 
		QString title = "Main MainWindowindow", QString iconPath = "", bool addDefaultContent = false);

    void setupMenuAndToolbar();
    void generateTestcases();
	bool writeToTestfiles(int i, string ifconnection, double a, double b);
	void restore();
	void solveGeneratedProblem(int , string, double&, double&, int&, int&);
	bool writeRawfiles(int i);
	void wholeTest();
	void solveGeneratedProblemForRobotNum(int i, string ifconnection, double & snapLength, double& visibilityLength, int& shortcut_times, int& smooth_nums, int robot_num);
	void openEnvironment_2();
	void readProblemFromFile(std::string filename);
	void runTest();
	void generateOptimalRatioTest();
void optimalRatioTest();
void optimalLossTest();
void addIntoEnv(Polygon_2 tp, int index);
Polygon_2 addNewNearPoly(std::vector<Polygon_2> exist_polys, std::vector<Polygon_2> all_created_polys);
Polygon_2 addNewNearTetris(std::vector<Polygon_2> exist_polys, std::vector<Polygon_2> all_created_polys);
bool isTwoPolyTooClose(Polygon_2 a, Polygon_2 b);
public slots:
    void openEnvironment();
	void overlayLattice();
	void locateBoundingLatticeCycle(bool doConnection = true);

    void createRandomProblem();
    void solveProblem();
    void singleTest();
	void animate();

	void fitView();

	void animatePath();

protected:

	void setupStatusBar();
  	void addNavigation(QGraphicsView*);
  	GraphicsViewNavigation* navigation;
  	QLabel* xycoord ;
	QApplication * m_pApp;
	QString	m_fileFolder;
	
    QToolBar * m_pMainToolBar;

    QAction * m_pOpenEnvAction;

    QAction * m_pOverlayLatticAction;
    QAction * m_pLocateBoundaryAction;
    QAction * m_pConnectGraphAction;

    QAction * m_pCreateAction;
	QAction * m_pSolveAction;
	QAction * m_pPlayAction;
	QAction * m_pTestAction;

	QLabel	* m_pNumberofRobotLabel;
	QLineEdit *	m_pLineEdit;

	QLabel	*	m_pMinDistLabel;
	QLineEdit *	m_pMinDistLineEdit;

    QAction * m_pFitScreenAction;

	QString	m_envFileName; 


	QGraphicsView* view;
    QGraphicsView* m_pView;
    QGraphicsScene m_scene;

	// Internal building state
	int	m_state;
	static const int BUILD_STATE_UNKNOWN = -1;
	static const int BUILD_STATE_MAP_OPEN = 0;
	static const int BUILD_STATE_LATTICE_OVERLAYED = 1;
	static const int BUILD_STATE_TRIMMED = 2;
	static const int BUILD_STATE_CONNECTION_FIXED = 3;

	// The environment
	double	m_radius;
    Polygon_2 m_boundingPoly;
    Polygon_2 m_boundingPoly2;
    Polygon2_list m_envPolyList;
    Polygon2_list m_envObsPolyList;
    Polygon2_list m_envPolyVoronoiList;
	Polygon2_list m_envObstaclePolyList;
	Polygon2_list m_envObsObstaclePolyList;

    AdvancedGraphicsItem<Polygon_2> * m_pBoundingPolyAGI;
    AdvancedGraphicsItem<Polygon_2> * m_pBoundingPolyAGI2;
    std::vector<AdvancedGraphicsItem<Polygon_2> *>  m_PolyAGIList;

	// Bounding rect
	QRectF m_boundingRect;

	// Roadmap
	Roadmap m_roadmap;

	// Number of robots
	int			m_numRobots;

	// Timer for animation
	QTimer *	m_pTimer;
	
	// Start and goal locations 
	vector<pair<double, double> >	m_svVec;
	vector<pair<double, double> >	m_gvVec;
	vector<pair<double, double> >   m_init_start;
	vector<pair<int, int> >			m_sgVec;
	bool m_hasInitStart;
	
	// Paths
	map<int, vector<pair<double, double>> >			m_pathMap;

	// Current 
	map<int, pair<double, double> >	m_currentStartMap;
	map<int, pair<double, double> >	m_currentGoalMap;
	map<int, vector<pair<double, double>>> m_currentPathMap;
	std::map<int, std::vector<double>> m_lengthListMap;
    std::map<int, std::vector<double>> m_lengthAdditiveListMap;

	// Vector of robot graphics item pointers
	vector<QGraphicsEllipseItem*>	m_pRobotItemVec;
	vector<QGraphicsSimpleTextItem*>	m_pTextItemVec;
	
	static const int ANI_STATE_FROM_START = 0;
	static const int ANI_STATE_GRID_PATH = 10;
	static const int ANI_STATE_TO_GOAL = 20;
	static const int ANI_STATE_COMPLETE = 30;

	// Shortest path related
	double m_shotestPathLength;
	vector<QGraphicsLineItem*> m_pPathLineItems;
	double m_largestSnapLength;

	int			m_aniState;			
	int			m_pathStep;
	double		m_aniStatePercent;

private:
	// Rendering
	void drawBasicEnvironment();

} ;

#endif //_MAIN_WINDOW_H_
