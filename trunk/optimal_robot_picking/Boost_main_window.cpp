/*
 *	Main window class implemenation. 
 *
 *  Created on: Jan 30, 2015
 *  Author: Jingjin Yu
 */

#include "Boost_main_window.h"
#include "Boost_helper_functions.h"
#include <QLabel>
#include <QMenuBar>
#include <QMenu>
#include <QToolBar>
#include <QToolButton>
#include <QFileDialog>
#include <QGraphicsView>
#include <QMainWindow>
#include <QApplication>
#include <QLabel>
#include <QFile>
#include <QFileInfo>
#include <QMenu>
#include <QMenuBar>
#include <QAction>
#include <QMessageBox>
#include <QStatusBar>
#include <QGraphicsView>
#include <QGLWidget>
#include <QTextStream>
#include <QSettings>
#include <QUrl>
#include <QDesktopWidget>
#include <QRegExp>
//#include <QSvgGenerator>
#include <QtCore>
#include <time.h>  
#include <QtOpenGL>
#if _MSC_VER == 1600
#include <qgl.h>
#endif

//#define SMALL_ENV


#include <iostream>
#include <ios>
#include <string>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp> 
#define _DEBUG

MainWindow::MainWindow( QApplication *pApp, QString& fileFolder, 
	QString title, QString iconPath, bool addDefaultContent)
:m_pApp(pApp), m_fileFolder(fileFolder), m_state(BUILD_STATE_UNKNOWN),QMainWindow()
{

	xycoord = new QLabel(" -0.00000 , -0.00000 ", this);
  	xycoord->setMinimumSize(xycoord->sizeHint());
  	xycoord->clear();

    m_pBoundingPolyAGI = 0;
	m_pBoundingPolyAGI2 = 0;

    // Set title
    setWindowTitle(title);

    // Set icon if the icon path is given
    if(iconPath.length() > 0){
        this->setWindowIcon(QIcon(iconPath));
    }


    // Check whether to display some default content
    if(addDefaultContent){
        resize(600, 400);
        QLabel * label = new QLabel(tr("Central Widget"));
        setCentralWidget(label);
        label->setAlignment(Qt::AlignCenter);
		m_pView->setMouseTracking(true);
    }
    else{
        m_pView = new QGraphicsView(this);
#if _MSC_VER == 1600
		m_pView->setViewport(new QGLWidget(QGLFormat(QGL::SampleBuffers)));
#endif

        m_scene.setItemIndexMethod(QGraphicsScene::NoIndex);
        m_scene.setSceneRect(-100, -100, 100, 100);
        m_pView->setScene(&m_scene);
        m_pView->setMouseTracking(true);

        this->view = m_pView;

        resize(1400, 1000);
        this->view->resize(1400, 1000);
        this->view->scale(1, 1);
        this->addNavigation(this->view);

        this->setupStatusBar();
        setCentralWidget(m_pView);
	}

	m_pView->setRenderHint(QPainter::Antialiasing);

	// Setup timer
	m_pTimer = new QTimer(this);
	connect(m_pTimer, SIGNAL(timeout()), this, SLOT(animatePath()));

	
	// Setup toolbar
    setupMenuAndToolbar();
}							

void MainWindow::addNavigation(QGraphicsView* graphicsView)
{
  navigation = new GraphicsViewNavigation();
  graphicsView->viewport()->installEventFilter(navigation);
  graphicsView->installEventFilter(navigation);
  QObject::connect(navigation, SIGNAL(mouseCoordinates(QString)),
		   xycoord, SLOT(setText(QString)));
  view = graphicsView;
}

void MainWindow::setupStatusBar()
{
	this->statusBar()->addWidget(new QLabel(this), 1);
	this->statusBar()->addWidget(xycoord, 0);
}

void MainWindow::setupMenuAndToolbar(){
	int iconSize = 40;
    m_pMainToolBar = new QToolBar(tr("Main Toolbar"), this);
    m_pMainToolBar->setIconSize(QSize(iconSize, iconSize));
    addToolBar(m_pMainToolBar);
    // m_pMainToolBar->setVisible(false);


    // Create the environment menu
    QMenu* fileMenu = menuBar()->addMenu(tr("&Environment"));

    // Open action
    m_pOpenEnvAction = new QAction(tr("&Open Environment"), this);
    m_pOpenEnvAction->setStatusTip(tr("Open a new envrionment"));
    m_pOpenEnvAction->setIcon(QIcon("./resources/open.png"));
    connect(m_pOpenEnvAction, SIGNAL(triggered()), this, SLOT(openEnvironment()));
    fileMenu->addAction(m_pOpenEnvAction);
    m_pMainToolBar->addAction(m_pOpenEnvAction);

	m_pMainToolBar->addSeparator();

    // Connect graph action
    m_pOverlayLatticAction = new QAction(tr("&Overlay Lattice Graph"), this);
    m_pOverlayLatticAction->setStatusTip(tr("Overlay covering lattice graph"));
    m_pOverlayLatticAction->setIcon(QIcon("./resources/hexgrid.png"));
    connect(m_pOverlayLatticAction, SIGNAL(triggered()), this, SLOT(overlayLattice()));
	fileMenu->addSeparator();
    fileMenu->addAction(m_pOverlayLatticAction);
    m_pMainToolBar->addAction(m_pOverlayLatticAction);
	m_pOverlayLatticAction->setDisabled(true); // Disabled at beginning 

    m_pLocateBoundaryAction = new QAction(tr("&Trim Lattice Graph"), this);
    m_pLocateBoundaryAction->setStatusTip(tr("Remove lattice parts outside the free space"));
    m_pLocateBoundaryAction->setIcon(QIcon("./resources/hexcycle.png"));
    connect(m_pLocateBoundaryAction, SIGNAL(triggered()), this, SLOT(locateBoundingLatticeCycle()));
    fileMenu->addAction(m_pLocateBoundaryAction);
    m_pMainToolBar->addAction(m_pLocateBoundaryAction);
	m_pLocateBoundaryAction->setDisabled(true); // Disabled at beginning 

	QWidget* empty3 = new QWidget();
	empty3->setFixedWidth(4);
	m_pMainToolBar->addWidget(empty3);

	m_pNumberofRobotLabel = new QLabel(this);
	m_pNumberofRobotLabel->setText(" Robots: ");
	m_pNumberofRobotLabel->setFixedHeight(iconSize-2);
	m_pMainToolBar->addWidget(m_pNumberofRobotLabel);

	m_pLineEdit = new QLineEdit(this);
	m_pLineEdit->setFixedWidth(30);
	m_pLineEdit->setFixedHeight(iconSize-2);
	m_pLineEdit->setText(QString("25"));
	m_pMainToolBar->addWidget(m_pLineEdit);

	QWidget* empty1 = new QWidget();
	empty1->setFixedWidth(4);
	m_pMainToolBar->addWidget(empty1);

	m_pMinDistLabel = new QLabel(this);
	m_pMinDistLabel->setText(" Spacing: ");
	m_pMinDistLabel->setFixedHeight(iconSize-2);
	m_pMainToolBar->addWidget(m_pMinDistLabel);

	m_pMinDistLineEdit = new QLineEdit(this);
	m_pMinDistLineEdit->setFixedWidth(30);
	m_pMinDistLineEdit->setFixedHeight(iconSize-2);
	m_pMinDistLineEdit->setText(QString("3"));
	m_pMainToolBar->addWidget(m_pMinDistLineEdit);

	QWidget* empty2 = new QWidget();
	empty2->setFixedWidth(4);
	m_pMainToolBar->addWidget(empty2);

	m_pCreateAction = new QAction(tr("&Create Random Problem"), this);
    m_pCreateAction->setStatusTip(tr("Create a random instance"));
    m_pCreateAction->setIcon(QIcon("./resources/sg.png"));
    connect(m_pCreateAction, SIGNAL(triggered()), this, SLOT(createRandomProblem()));
    fileMenu->addAction(m_pCreateAction);
    m_pMainToolBar->addAction(m_pCreateAction);
	m_pCreateAction->setDisabled(false); // Disabled at beginning 

    m_pSolveAction = new QAction(tr("&Solve"), this);
    m_pSolveAction->setStatusTip(tr("Solve a randomly created instance"));
    m_pSolveAction->setIcon(QIcon("./resources/solution.png"));
    connect(m_pSolveAction, SIGNAL(triggered()), this, SLOT(solveProblem()));
    fileMenu->addAction(m_pSolveAction);
    m_pMainToolBar->addAction(m_pSolveAction);
	m_pSolveAction->setDisabled(false); // Disabled at beginning 

    m_pPlayAction = new QAction(tr("&Animate"), this);
    m_pPlayAction->setStatusTip(tr("Animate the solution"));
    m_pPlayAction->setIcon(QIcon("./resources/play.png"));
    connect(m_pPlayAction, SIGNAL(triggered()), this, SLOT(animate()));
    fileMenu->addAction(m_pPlayAction);
    m_pMainToolBar->addAction(m_pPlayAction);
	m_pPlayAction->setDisabled(true); // Disabled at beginning 

    // Fit to screen action
	m_pFitScreenAction = new QAction(tr("&Fit content to view"), this);
    m_pFitScreenAction->setStatusTip(tr("Fit content to view"));
    m_pFitScreenAction->setIcon(QIcon("./resources/fitview.png"));
    connect(m_pFitScreenAction, SIGNAL(triggered()), this, SLOT(fitView()));
	fileMenu->addSeparator();
    fileMenu->addAction(m_pFitScreenAction);
    m_pMainToolBar->addAction(m_pFitScreenAction);

    // m_pTestAction = new QAction(tr("&Test"), this);
    // m_pTestAction->setStatusTip(tr("Run some test"));
    // m_pTestAction->setIcon(QIcon("./resources/test.jpg"));
    // connect(m_pTestAction, SIGNAL(triggered()), this, SLOT(singleTest()));
    // fileMenu->addAction(m_pTestAction);
    // m_pMainToolBar->addAction(m_pTestAction);
    // m_pTestAction->setDisabled(false); // Disabled at beginning 
}

void MainWindow::readProblemFromFile(std::string filename) {
	m_numRobots = 1;
	// Clean up
	m_scene.clear();
	m_boundingPoly.clear();
	m_boundingPoly2.clear();
	m_envPolyList.clear();
	m_envObsPolyList.clear();
	m_PolyAGIList.clear();

	// Reading in the environment, first the raidus of the (disc) robot
	std::ifstream ifs(filename);
	double radius;
	ifs >> radius;

	// Then the number of obstacle polygons
	int numberOfPolygons;
	ifs >> numberOfPolygons;
	m_numRobots = numberOfPolygons;
	// Then read in all obstacle polygons and compute the configuration space for a single disc
	for (int i = 0; i < numberOfPolygons; i++) {
		// Load polygon
		Polygon_2 tp;
		int numberOfVertex;
		ifs >> numberOfVertex;
		//Point_2 firstOne_1;
		for (int j = 0; j < numberOfVertex; j++) {
			Point_2 p;
			double p_x, p_y;
			ifs >> p_x >> p_y;
			// if(j == 0){
			//      		firstOne_1.set<0>(p_x);
			//      		firstOne_1.set<1>(p_y);
			//      	}
			p.set<0>(p_x);
			p.set<1>(p_y);
			bg::append(tp.outer(), p);
		}

		// bg::append(tp.outer(), firstOne_1);
		bg::correct(tp);
		m_envPolyList.push_back(tp);


		// Add raw obstacle to scene and set fill to be true with fill transparency
		AdvancedGraphicsItem<Polygon_2>* pAGI = new AdvancedGraphicsItem<Polygon_2>(&(m_envPolyList.back()));

		// m_scene.addItem(pAGI);
		pAGI->m_bShowEdge = true;
		pAGI->m_bShowVertices = true;
		pAGI->m_bFill = false;
		pAGI->m_fillBrush = QColor(16, 16, 16, 192);
		m_PolyAGIList.push_back(pAGI);

		// Computing the Minkowski sum
		int split_num = 8;
		Polygon_2 ep = growPolygonByRadius(tp, radius, split_num);
		bg::correct(ep);
		//split_num = 4;
		//Polygon_2 vp = growPolygonByRadius(tp, radius, split_num);
		//bg::correct(vp);

		m_envPolyVoronoiList.push_back(ep);
		m_envObsPolyList.push_back(ep);

		// Add the configuration space obstacle to the scene
		pAGI = new AdvancedGraphicsItem<Polygon_2>(&(m_envObsPolyList.back()));

		// m_scene.addItem(pAGI);
		pAGI->m_bFill = true;
		pAGI->m_edgePen = QPen(Qt::gray, 0.25, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
		pAGI->m_vertexPen = QPen(Qt::black, 0.5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
		m_PolyAGIList.push_back(pAGI);
	}

	// Then read the bounding rectangle (configuration space)
	int numberOfBoundingVertex;

	ifs >> numberOfBoundingVertex;
	for (int j = 0; j < numberOfBoundingVertex; j++) {
		Point_2 p;
		double p_x, p_y;
		ifs >> p_x >> p_y;

		p.set<0>(p_x);
		p.set<1>(p_y);
		bg::append(m_boundingPoly.outer(), p);
	}
	bg::correct(m_boundingPoly);

	m_envPolyVoronoiList.push_back(m_boundingPoly);
	// Add to scence

	m_pBoundingPolyAGI = new AdvancedGraphicsItem<Polygon_2>(&m_boundingPoly);
	m_pBoundingPolyAGI->m_bFill = false;
	m_pBoundingPolyAGI->m_bShowVertices = false;
	m_pBoundingPolyAGI->m_bShowEdge = true;
	m_pBoundingPolyAGI->m_edgePen = QPen(Qt::gray, 0.25, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
	m_PolyAGIList.push_back(m_pBoundingPolyAGI);
	// m_scene.addItem(m_pBoundingPolyAGI);

	// Then compute the outside bounding rectangle
	double x1, y1, x2, y2;
	std::vector<Point_2> const& points = m_boundingPoly.outer();

	x1 = points[0].get<0>(); y1 = points[0].get<1>();
	x2 = points[2].get<0>(); y2 = points[2].get<1>();
	bg::append(m_boundingPoly2.outer(), Point_2(x1 - radius, y1 - radius));
	bg::append(m_boundingPoly2.outer(), Point_2(x1 - radius, y2 + radius));
	bg::append(m_boundingPoly2.outer(), Point_2(x2 + radius, y2 + radius));
	bg::append(m_boundingPoly2.outer(), Point_2(x2 + radius, y1 - radius));
	//	bg::append(m_boundingPoly2.outer(), Point_2(x1 - radius, y1 - radius));
	bg::correct(m_boundingPoly2);
	// Add to scene
	m_pBoundingPolyAGI2 = new AdvancedGraphicsItem<Polygon_2>(&m_boundingPoly2);
	m_pBoundingPolyAGI2->m_bFill = false;
	m_pBoundingPolyAGI2->m_bShowVertices = false;
	m_pBoundingPolyAGI2->m_bShowEdge = true;
	m_pBoundingPolyAGI2->m_edgePen = QPen(Qt::black, 0.25, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
	m_PolyAGIList.push_back(m_pBoundingPolyAGI2);
	// m_scene.addItem(m_pBoundingPolyAGI2);

	m_radius = radius;

	drawBasicEnvironment();

	// Do roadmap building setup
	m_roadmap.buildRoadmap(&m_envObsPolyList, &m_envPolyList, &m_boundingPoly, &m_envPolyVoronoiList, radius, m_scene);
	// m_roadmap.addToScene(m_scene);

	m_boundingRect = QRectF(x1 - radius * 4, y1 - radius * 4, x2 + radius * 4, y2 + radius * 4);

	// Fit to the view
	fitView();
	ifs.close();
	// Enable the actions for building the graph
	m_state = BUILD_STATE_MAP_OPEN;
	m_pOverlayLatticAction->setEnabled(true);
	m_pLocateBoundaryAction->setEnabled(true);
	m_pCreateAction->setEnabled(true);
	m_pSolveAction->setEnabled(true);
	m_pPlayAction->setEnabled(false);
}

void MainWindow::openEnvironment(){
    //openEnvironment_2();
	 
//	Open file dialog for file selection
    QString m_fileNameq = QFileDialog::getOpenFileName(this, tr("Open Environment Description File"), "", tr("Files (*.*)"));
    if(m_fileNameq == 0 || m_fileNameq.length() < 0) return;

	QStringList qsl = m_fileNameq.split("/");
	m_envFileName = qsl[qsl.size()-1].split(".")[0];
	std::cout<<qPrintable(m_fileNameq);
    m_fileName = m_fileNameq.toStdString();
    std::cout<<m_fileName;
	m_numRobots = 1;
	// Clean up
    m_scene.clear();
    m_boundingPoly.clear();
	m_boundingPoly2.clear();
	m_envPolyList.clear();
	m_envObsPolyList.clear();
	m_PolyAGIList.clear(); 

	// Reading in the environment, first the raidus of the (disc) robot
    std::ifstream ifs(qPrintable(m_fileNameq));
    double radius;
    ifs >> radius;

	// Then the number of obstacle polygons
    int numberOfPolygons;
    ifs >> numberOfPolygons;
	
	// Then read in all obstacle polygons and compute the configuration space for a single disc
    for(int i = 0; i < numberOfPolygons; i ++){
        // Load polygon
        Polygon_2 tp;
        int numberOfVertex;
        ifs >> numberOfVertex;
		//Point_2 firstOne_1;
        for(int j = 0;j < numberOfVertex;j++){
        	Point_2 p;
        	double p_x, p_y;
        	ifs >> p_x >> p_y;
			// if(j == 0){
   //      		firstOne_1.set<0>(p_x);
   //      		firstOne_1.set<1>(p_y);
   //      	}
        	p.set<0>(p_x);
        	p.set<1>(p_y);
        	bg::append(tp.outer(), p);
        }
       // bg::append(tp.outer(), firstOne_1);
		bg::correct(tp);
        m_envPolyList.push_back(tp);


		// Add raw obstacle to scene and set fill to be true with fill transparency
        AdvancedGraphicsItem<Polygon_2>* pAGI = new AdvancedGraphicsItem<Polygon_2>(&(m_envPolyList.back()));
        
        // m_scene.addItem(pAGI);
        pAGI->m_bShowEdge = true;
        pAGI->m_bShowVertices = true;
        pAGI->m_bFill = false;
        pAGI->m_fillBrush = QColor(16,16,16,192);
		m_PolyAGIList.push_back(pAGI);

        // Computing the Minkowski sum
        int split_num = 8;
        Polygon_2 ep = growPolygonByRadius(tp, radius, split_num);
		bg::correct(ep);
        //split_num = 4;
        //Polygon_2 vp = growPolygonByRadius(tp, radius, split_num);
        //bg::correct(vp);

        m_envPolyVoronoiList.push_back(ep);
		m_envObsPolyList.push_back(ep);

		// Add the configuration space obstacle to the scene
        pAGI = new AdvancedGraphicsItem<Polygon_2>(&(m_envObsPolyList.back()));
        
        // m_scene.addItem(pAGI);
        pAGI->m_bFill = true;
        pAGI->m_edgePen = QPen(Qt::gray, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
        pAGI->m_vertexPen = QPen(Qt::black, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
		m_PolyAGIList.push_back(pAGI);
    }

	// Then read the bounding rectangle (configuration space)
    int numberOfBoundingVertex;
    
        ifs >> numberOfBoundingVertex;
        for(int j = 0;j < numberOfBoundingVertex;j++){
        	Point_2 p;
        	double p_x, p_y;
        	ifs >> p_x >> p_y;
        	
        	p.set<0>(p_x);
        	p.set<1>(p_y);
        	bg::append(m_boundingPoly.outer(), p);
        }
        bg::correct(m_boundingPoly);
	
	int numberOfObstacles = 0;
	ifs >> numberOfObstacles;
	if (numberOfObstacles > 0) {
		for (int i = 0; i < numberOfObstacles; i++) {
			// Load polygon
			Polygon_2 tp;
			int numberOfVertex;
			ifs >> numberOfVertex;
			//Point_2 firstOne_1;
			for (int j = 0; j < numberOfVertex; j++) {
				Point_2 p;
				double p_x, p_y;
				ifs >> p_x >> p_y;
				// if(j == 0){
				//      		firstOne_1.set<0>(p_x);
				//      		firstOne_1.set<1>(p_y);
				//      	}
				p.set<0>(p_x);
				p.set<1>(p_y);
				bg::append(tp.outer(), p);
			}
			// bg::append(tp.outer(), firstOne_1);
			bg::correct(tp);
			m_envObstaclePolyList.push_back(tp);

			// Add raw obstacle to scene and set fill to be true with fill transparency
			AdvancedGraphicsItem<Polygon_2>* pAGI = new AdvancedGraphicsItem<Polygon_2>(&(m_envObstaclePolyList.back()));

			// m_scene.addItem(pAGI);
			pAGI->m_bShowEdge = true;
			pAGI->m_bShowVertices = true;
			pAGI->m_bFill = false;
			pAGI->m_fillBrush = QColor(16, 16, 16, 192);
			m_PolyAGIList.push_back(pAGI);

			// Computing the Minkowski sum
			int split_num = 8;
			Polygon_2 ep = growPolygonByRadius(tp, radius, split_num);
			bg::correct(ep);
			//split_num = 4;
			//Polygon_2 vp = growPolygonByRadius(tp, radius, split_num);
			//bg::correct(vp);
			m_envObsObstaclePolyList.push_back(ep);
			//m_envPolyVoronoiList.push_back(ep);
			//m_envObsPolyList.push_back(ep);

			// Add the configuration space obstacle to the scene
			pAGI = new AdvancedGraphicsItem<Polygon_2>(&(m_envObsObstaclePolyList.back()));

			// m_scene.addItem(pAGI);
			pAGI->m_bFill = true;
			pAGI->m_edgePen = QPen(Qt::gray, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
			pAGI->m_vertexPen = QPen(Qt::black, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
			m_PolyAGIList.push_back(pAGI);
		}
	}
	m_envPolyVoronoiList.push_back(m_boundingPoly);
	// Add to scence

    m_pBoundingPolyAGI = new AdvancedGraphicsItem<Polygon_2>(&m_boundingPoly);
    m_pBoundingPolyAGI->m_bFill = false;
    m_pBoundingPolyAGI->m_bShowVertices = false;
    m_pBoundingPolyAGI->m_bShowEdge = true;
	m_pBoundingPolyAGI->m_edgePen = QPen(Qt::gray, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    m_PolyAGIList.push_back(m_pBoundingPolyAGI);
    // m_scene.addItem(m_pBoundingPolyAGI);

	// Then compute the outside bounding rectangle
	double x1, y1, x2, y2;
	std::vector<Point_2> const& points = m_boundingPoly.outer();

	x1 = points[0].get<0>(); y1 = points[0].get<1>();
	x2 = points[2].get<0>(); y2 = points[2].get<1>();
	bg::append(m_boundingPoly2.outer(), Point_2(x1 - radius, y1 - radius));
	bg::append(m_boundingPoly2.outer(), Point_2(x1 - radius, y2 + radius));
	bg::append(m_boundingPoly2.outer(), Point_2(x2 + radius, y2 + radius));
	bg::append(m_boundingPoly2.outer(), Point_2(x2 + radius, y1 - radius));
//	bg::append(m_boundingPoly2.outer(), Point_2(x1 - radius, y1 - radius));
    bg::correct(m_boundingPoly2);
	// Add to scene
    m_pBoundingPolyAGI2 = new AdvancedGraphicsItem<Polygon_2>(&m_boundingPoly2);
    m_pBoundingPolyAGI2->m_bFill = false;
    m_pBoundingPolyAGI2->m_bShowVertices = false;
    m_pBoundingPolyAGI2->m_bShowEdge = true;
	m_pBoundingPolyAGI2->m_edgePen = QPen(Qt::black, 0.25, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    m_PolyAGIList.push_back(m_pBoundingPolyAGI2);
    // m_scene.addItem(m_pBoundingPolyAGI2);

	m_radius = radius;

	drawBasicEnvironment();

	// Do roadmap building setup
	m_roadmap.buildRoadmap(&m_envObsPolyList, &m_envPolyList, &m_boundingPoly, &m_envPolyVoronoiList, radius, m_scene, &m_envObsObstaclePolyList, &m_envObstaclePolyList);
	// m_roadmap.addToScene(m_scene);

	m_boundingRect = QRectF(x1 - radius*4, y1 - radius*4, x2 + radius*4, y2 + radius*4);

    // Fit to the view
    fitView();

	// Enable the actions for building the graph
	m_state = BUILD_STATE_MAP_OPEN;
	m_pOverlayLatticAction->setEnabled(true);
    m_pLocateBoundaryAction->setEnabled(true);
	m_pCreateAction->setEnabled(true);
	m_pSolveAction->setEnabled(true);
	m_pPlayAction->setEnabled(false);

}

void MainWindow::openEnvironment_2() {
	// Open file dialog for file selection
	QString m_fileNameq = QFileDialog::getOpenFileName(this, tr("Open Environment Description File"), "", tr("Files (*.*)"));
	if (m_fileNameq == 0 || m_fileNameq.length() < 0) return;

	QStringList qsl = m_fileNameq.split("/");
	m_envFileName = qsl[qsl.size() - 1].split(".")[0];
	std::cout << qPrintable(m_fileNameq);
	m_fileName = m_fileNameq.toStdString();
	std::cout << m_fileName;
	m_numRobots = m_pLineEdit->text().toInt();
	// Clean up
	m_scene.clear();
	m_boundingPoly.clear();
	m_boundingPoly2.clear();
	m_envPolyList.clear();
	m_envObsPolyList.clear();
	m_PolyAGIList.clear(); /* The element pointers are gone when the scene object is cleared */

						   // Reading in the environment, first the raidus of the (disc) robot
	std::ifstream ifs(qPrintable(m_fileNameq));
	double x_min, x_max, y_min, y_max;
	double radius;
	double edgeLength;
	ifs >> x_min;
	ifs >> x_max;
	ifs >> y_min;
	ifs >> y_max;
	ifs >> edgeLength;
	radius = edgeLength * 0.43;
	Point_2 left_top, right_top, left_bottom, right_bottom;
	left_top.set<0>(x_min);
	left_top.set<1>(y_min);
	right_top.set<0>(x_max);
	right_top.set<1>(y_min);
	left_bottom.set<0>(x_min);
	left_bottom.set<1>(y_max);
	right_bottom.set<0>(x_max);
	right_bottom.set<1>(y_max);
	bg::append(m_boundingPoly.outer(), left_top);
	bg::append(m_boundingPoly.outer(), right_top);
	bg::append(m_boundingPoly.outer(), right_bottom);
	bg::append(m_boundingPoly.outer(), left_bottom);

	bg::correct(m_boundingPoly);

	m_envPolyVoronoiList.push_back(m_boundingPoly);
	// Add to scence

	m_pBoundingPolyAGI = new AdvancedGraphicsItem<Polygon_2>(&m_boundingPoly);
	m_pBoundingPolyAGI->m_bFill = false;
	m_pBoundingPolyAGI->m_bShowVertices = false;
	m_pBoundingPolyAGI->m_bShowEdge = true;
	m_pBoundingPolyAGI->m_edgePen = QPen(Qt::gray, 0.25, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
	m_PolyAGIList.push_back(m_pBoundingPolyAGI);
	// m_scene.addItem(m_pBoundingPolyAGI);

	// Then compute the outside bounding rectangle
	double x1, y1, x2, y2;
	std::vector<Point_2> const& points = m_boundingPoly.outer();

	x1 = points[0].get<0>(); y1 = points[0].get<1>();
	x2 = points[2].get<0>(); y2 = points[2].get<1>();
	bg::append(m_boundingPoly2.outer(), Point_2(x1 - radius, y1 - radius));
	bg::append(m_boundingPoly2.outer(), Point_2(x1 - radius, y2 + radius));
	bg::append(m_boundingPoly2.outer(), Point_2(x2 + radius, y2 + radius));
	bg::append(m_boundingPoly2.outer(), Point_2(x2 + radius, y1 - radius));
	//	bg::append(m_boundingPoly2.outer(), Point_2(x1 - radius, y1 - radius));
	bg::correct(m_boundingPoly2);
	// Add to scene
	m_pBoundingPolyAGI2 = new AdvancedGraphicsItem<Polygon_2>(&m_boundingPoly2);
	m_pBoundingPolyAGI2->m_bFill = false;
	m_pBoundingPolyAGI2->m_bShowVertices = false;
	m_pBoundingPolyAGI2->m_bShowEdge = true;
	m_pBoundingPolyAGI2->m_edgePen = QPen(Qt::black, 0.25, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
	m_PolyAGIList.push_back(m_pBoundingPolyAGI2);
	// m_scene.addItem(m_pBoundingPolyAGI2);

	m_radius = radius;












	// Then the number of obstacle polygons
	int numberOfPolygons;
	ifs >> numberOfPolygons;

	// Then read in all obstacle polygons and compute the configuration space for a single disc
	for (int i = 0; i < numberOfPolygons; i++) {
		// Load polygon
		Polygon_2 tp;
		int numberOfVertex;
		ifs >> numberOfVertex;
		//Point_2 firstOne_1;
		for (int j = 0; j < numberOfVertex; j++) {
			Point_2 p;
			double p_x, p_y;
			ifs >> p_x >> p_y;
			// if(j == 0){
			//      		firstOne_1.set<0>(p_x);
			//      		firstOne_1.set<1>(p_y);
			//      	}
			p.set<0>(p_x);
			p.set<1>(p_y);
			bg::append(tp.outer(), p);
		}
		// bg::append(tp.outer(), firstOne_1);
		bg::correct(tp);
		m_envPolyList.push_back(tp);


		// Add raw obstacle to scene and set fill to be true with fill transparency
		AdvancedGraphicsItem<Polygon_2>* pAGI = new AdvancedGraphicsItem<Polygon_2>(&(m_envPolyList.back()));

		// m_scene.addItem(pAGI);
		pAGI->m_bShowEdge = true;
		pAGI->m_bShowVertices = true;
		pAGI->m_bFill = false;
		pAGI->m_fillBrush = QColor(16, 16, 16, 192);
		m_PolyAGIList.push_back(pAGI);

		// Computing the Minkowski sum
		int split_num = 8;
		Polygon_2 ep = growPolygonByRadius(tp, radius, split_num);
		bg::correct(ep);
		//split_num = 4;
		//Polygon_2 vp = growPolygonByRadius(tp, radius, split_num);
		//bg::correct(vp);

		m_envPolyVoronoiList.push_back(ep);
		m_envObsPolyList.push_back(ep);

		// Add the configuration space obstacle to the scene
		pAGI = new AdvancedGraphicsItem<Polygon_2>(&(m_envObsPolyList.back()));

		// m_scene.addItem(pAGI);
		pAGI->m_bFill = true;
		pAGI->m_edgePen = QPen(Qt::gray, 0.25, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
		pAGI->m_vertexPen = QPen(Qt::black, 0.5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
		m_PolyAGIList.push_back(pAGI);
	}
	double num_robot;
	ifs >> num_robot;
	m_numRobots = num_robot;
	std::vector<pair<double, double>> init_start;
	double init_x, init_y;
	pair<double, double> init_pt;
	for (int i = 0; i < m_numRobots; i++) {
		ifs >> init_x >> init_y;
		init_pt.first = init_x;
		init_pt.second = init_y;
		init_start.push_back(init_pt);
	}
	m_init_start = init_start;
	m_hasInitStart = true;

	QFont font;
	QPainterPath path;
	font.setPointSizeF(m_radius / 1.5);
	font.setBold(true);
	/*
	for (int r = 0; r < m_numRobots; r++) {
		// Robots at current (start) locations
		QGraphicsEllipseItem *prs = m_scene.addEllipse(m_svVec[r].first - m_radius, m_svVec[r].second - m_radius, m_radius * 2 - 0.2, m_radius * 2 - 0.2,
			QPen(Qt::black, 0.2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin),
			QBrush(Qt::blue));
		prs->setZValue(10);
		m_pRobotItemVec.push_back(prs);

		// Labels for robots at current (start) locations
		QGraphicsSimpleTextItem *ti = m_scene.addSimpleText(QString::number(r + 1), font);
		ti->setPos(m_svVec[r].first - m_radius / 2 * (3.5 / (m_radius + 1)) + (r < 9 ? m_radius / 6 * (2.2 / (m_radius - 0.3)) : (r > 99 ? -m_radius / 6 * (2.2 / (m_radius - 0.3)) : 0)), m_svVec[r].second - m_radius / 2 * (2. / (m_radius - 0.5)));
		ti->setPen(QPen(QColor(Qt::white), 0.15, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
		ti->setZValue(15);
		m_pTextItemVec.push_back(ti);
		*/


		drawBasicEnvironment();

		// Do roadmap building setup
		m_roadmap.buildRoadmap(&m_envObsPolyList, &m_envPolyList, &m_boundingPoly, &m_envPolyVoronoiList, radius, m_scene);
		// m_roadmap.addToScene(m_scene);

		m_boundingRect = QRectF(x1 - radius * 4, y1 - radius * 4, x2 + radius * 4, y2 + radius * 4);

		// Fit to the view
		//  fitView();

		// Enable the actions for building the graph
		m_state = BUILD_STATE_MAP_OPEN;
		m_pOverlayLatticAction->setEnabled(true);
		m_pLocateBoundaryAction->setEnabled(true);
		m_pCreateAction->setEnabled(false);
		m_pSolveAction->setEnabled(false);
		m_pPlayAction->setEnabled(false);

	}


void MainWindow::restore(){
    m_scene.clear();
    m_boundingPoly.clear();
    m_boundingPoly2.clear();
    m_envPolyList.clear();
    m_envObsPolyList.clear();
    m_PolyAGIList.clear(); /* The element pointers are gone when the scene object is cleared */
	if (m_fileName.length() <= 0) {
		std::cout << "filename doesnot exist" << endl;
		return;
	}
	std::cout << "fileName:" << m_fileName;
    // Reading in the environment, first the raidus of the (disc) robot
    std::ifstream ifs(m_fileName);
    double radius;
    ifs >> radius;

    // Then the number of obstacle polygons
    int numberOfPolygons;
    ifs >> numberOfPolygons;
    
    // Then read in all obstacle polygons and compute the configuration space for a single disc
    for(int i = 0; i < numberOfPolygons; i ++){
        // Load polygon
        Polygon_2 tp;
        int numberOfVertex;
        ifs >> numberOfVertex;
        //Point_2 firstOne_1;
        for(int j = 0;j < numberOfVertex;j++){
            Point_2 p;
            double p_x, p_y;
            ifs >> p_x >> p_y;
            // if(j == 0){
   //           firstOne_1.set<0>(p_x);
   //           firstOne_1.set<1>(p_y);
   //       }
            p.set<0>(p_x);
            p.set<1>(p_y);
            bg::append(tp.outer(), p);
        }
       // bg::append(tp.outer(), firstOne_1);
        bg::correct(tp);
        m_envPolyList.push_back(tp);


        // Add raw obstacle to scene and set fill to be true with fill transparency
        AdvancedGraphicsItem<Polygon_2>* pAGI = new AdvancedGraphicsItem<Polygon_2>(&(m_envPolyList.back()));
        
        // m_scene.addItem(pAGI);
        pAGI->m_bShowEdge = true;
        pAGI->m_bShowVertices = true;
        pAGI->m_bFill = false;
        pAGI->m_fillBrush = QColor(16,16,16,192);
        m_PolyAGIList.push_back(pAGI);

        // Computing the Minkowski sum
        int split_num = 8;
        Polygon_2 ep = growPolygonByRadius(tp, radius, split_num);
        bg::correct(ep);
        //split_num = 4;
        //Polygon_2 vp = growPolygonByRadius(tp, radius, split_num);
        //bg::correct(vp);

        m_envPolyVoronoiList.push_back(ep);
        m_envObsPolyList.push_back(ep);

        // Add the configuration space obstacle to the scene
        pAGI = new AdvancedGraphicsItem<Polygon_2>(&(m_envObsPolyList.back()));
        
        // m_scene.addItem(pAGI);
        pAGI->m_bFill = true;
        pAGI->m_edgePen = QPen(Qt::gray, 0.25, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
        pAGI->m_vertexPen = QPen(Qt::black, 0.5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
        m_PolyAGIList.push_back(pAGI);
    }

    // Then read the bounding rectangle (configuration space)
    int numberOfBoundingVertex;
    
        ifs >> numberOfBoundingVertex;
        for(int j = 0;j < numberOfBoundingVertex;j++){
            Point_2 p;
            double p_x, p_y;
            ifs >> p_x >> p_y;
            
            p.set<0>(p_x);
            p.set<1>(p_y);
            bg::append(m_boundingPoly.outer(), p);
        }
        bg::correct(m_boundingPoly);

    m_envPolyVoronoiList.push_back(m_boundingPoly);
    // Add to scence

    m_pBoundingPolyAGI = new AdvancedGraphicsItem<Polygon_2>(&m_boundingPoly);
    m_pBoundingPolyAGI->m_bFill = false;
    m_pBoundingPolyAGI->m_bShowVertices = false;
    m_pBoundingPolyAGI->m_bShowEdge = true;
    m_pBoundingPolyAGI->m_edgePen = QPen(Qt::gray, 0.25, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    m_PolyAGIList.push_back(m_pBoundingPolyAGI);
    // m_scene.addItem(m_pBoundingPolyAGI);

    // Then compute the outside bounding rectangle
    double x1, y1, x2, y2;
    std::vector<Point_2> const& points = m_boundingPoly.outer();

    x1 = points[0].get<0>(); y1 = points[0].get<1>();
    x2 = points[2].get<0>(); y2 = points[2].get<1>();
    bg::append(m_boundingPoly2.outer(), Point_2(x1 - radius, y1 - radius));
    bg::append(m_boundingPoly2.outer(), Point_2(x1 - radius, y2 + radius));
    bg::append(m_boundingPoly2.outer(), Point_2(x2 + radius, y2 + radius));
    bg::append(m_boundingPoly2.outer(), Point_2(x2 + radius, y1 - radius));
//  bg::append(m_boundingPoly2.outer(), Point_2(x1 - radius, y1 - radius));
    bg::correct(m_boundingPoly2);
    // Add to scene
    m_pBoundingPolyAGI2 = new AdvancedGraphicsItem<Polygon_2>(&m_boundingPoly2);
    m_pBoundingPolyAGI2->m_bFill = false;
    m_pBoundingPolyAGI2->m_bShowVertices = false;
    m_pBoundingPolyAGI2->m_bShowEdge = true;
    m_pBoundingPolyAGI2->m_edgePen = QPen(Qt::black, 0.25, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    m_PolyAGIList.push_back(m_pBoundingPolyAGI2);
    // m_scene.addItem(m_pBoundingPolyAGI2);

    m_radius = radius;

    drawBasicEnvironment();

    // Do roadmap building setup
    m_roadmap.buildRoadmap(&m_envObsPolyList, &m_envPolyList, &m_boundingPoly, &m_envPolyVoronoiList, radius, m_scene);
    // m_roadmap.addToScene(m_scene);

    m_boundingRect = QRectF(x1 - radius*4, y1 - radius*4, x2 + radius*4, y2 + radius*4);

    // Fit to the view
  //  fitView();

    // Enable the actions for building the graph
    m_state = BUILD_STATE_MAP_OPEN;
    m_pOverlayLatticAction->setEnabled(true);
    m_pLocateBoundaryAction->setEnabled(true);
    m_pCreateAction->setEnabled(false);
    m_pSolveAction->setEnabled(false);
    m_pPlayAction->setEnabled(false);

}


void MainWindow::drawBasicEnvironment(){
	m_PolyAGIList.clear();
	for(Polygon2_list::iterator pit = m_envPolyList.begin(); pit != m_envPolyList.end(); pit++){
		// Add raw obstacle to scene and set fill to be true with fill transparency
        AdvancedGraphicsItem<Polygon_2>* pAGI = new AdvancedGraphicsItem<Polygon_2>(&(*pit));
        pAGI->m_bShowEdge = true;
        pAGI->m_bShowVertices = false;
		pAGI->m_edgePen = QPen(Qt::red, 1.25, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
        pAGI->m_bFill = true;
        pAGI->m_fillBrush = QColor(16,16,16,192);
		m_PolyAGIList.push_back(pAGI);
        
    }

	for(Polygon2_list::iterator pit = m_envObsPolyList.begin(); pit != m_envObsPolyList.end(); pit++){
		// Add the configuration space obstacle to the scene
        AdvancedGraphicsItem<Polygon_2>* pAGI = new AdvancedGraphicsItem<Polygon_2>(&(*pit));
        pAGI->m_bFill = false;
		pAGI->m_bShowEdge = true;
		pAGI->m_bShowVertices = true;
        pAGI->m_edgePen = QPen(Qt::red, 0.25, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
        pAGI->m_vertexPen = QPen(Qt::black, 0.5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
		m_PolyAGIList.push_back(pAGI);

    }

	// Add to scence
    m_pBoundingPolyAGI = new AdvancedGraphicsItem<Polygon_2>(&m_boundingPoly);
    m_pBoundingPolyAGI->m_bFill = false;
    m_pBoundingPolyAGI->m_bShowVertices = false;
    m_pBoundingPolyAGI->m_bShowEdge = true;
	m_pBoundingPolyAGI->m_edgePen = QPen(Qt::gray, 0.25, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    m_PolyAGIList.push_back(m_pBoundingPolyAGI);

	// Add to scene
    m_pBoundingPolyAGI2 = new AdvancedGraphicsItem<Polygon_2>(&m_boundingPoly2);
    m_pBoundingPolyAGI2->m_bFill = false;
    m_pBoundingPolyAGI2->m_bShowVertices = false;
    m_pBoundingPolyAGI2->m_bShowEdge = true;
	m_pBoundingPolyAGI2->m_edgePen = QPen(Qt::black, 0.25, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    m_PolyAGIList.push_back(m_pBoundingPolyAGI2);

	for(std::vector<AdvancedGraphicsItem<Polygon_2> *>::iterator pagiItor = m_PolyAGIList.begin();
		pagiItor != m_PolyAGIList.end(); pagiItor ++){
        AdvancedGraphicsItem<Polygon_2>* pAGI = *pagiItor;
        m_scene.addItem(pAGI);
    }
}

// get the total distance of removing the objects using greedy algorithm
void MainWindow::overlayLattice(){
	m_roadmap.declutterUsingGreedy();
	int numberOfPolygons = m_pLineEdit->text().toInt();
	m_roadmap.declutterUsingTruncatedTree(numberOfPolygons);
	//m_roadmap.declutterMultiExitSeparateGreedy();
	//m_roadmap.declutterMultiExitGreedy();
	//m_roadmap.declutterUsingMultipleGreedy(numberOfPolygons);
	//m_roadmap.declutterUsingParticleGreedy(numberOfPolygons);
	/*  
	m_scene.clear();
	m_roadmap.buildHexgaonLattice();
	drawBasicEnvironment();
	m_roadmap.drawHexagonLattice(m_scene, true);
   */
   // m_roadmap.drawVoronoiDiagram(m_scene, true);
	// Set build state and enable the actions for building the graph
	m_state = BUILD_STATE_LATTICE_OVERLAYED;
	m_pOverlayLatticAction->setEnabled(false);
}

void MainWindow::optimalLossTest(){
	m_scene.clear();
	//m_roadmap.buildHexgaonLattice();
	m_roadmap.buildRectGridLattice();
	drawBasicEnvironment();
        m_roadmap.drawVertexIds(m_scene);

	m_roadmap.drawHexagonLattice(m_scene, true);
    m_roadmap.optimalLossTest();
}

void MainWindow::locateBoundingLatticeCycle(bool doConnection){
	// Clean up
	m_roadmap.declutterUsingGreedy();
	//int numberOfPolygons = m_pLineEdit->text().toInt();
	//m_roadmap.declutterUsingMultiExitTruncatedTree(numberOfPolygons);
	/*  
	std::vector<int> seq;
	double length;
	seq.push_back(4);
	seq.push_back(2);
	seq.push_back(0);
	seq.push_back(5);
	seq.push_back(6);
	seq.push_back(1);
	seq.push_back(3);
	length = m_roadmap.declutterUsingSequence(seq);
	std::cout << "length:" << length << std::endl;
	*/
	//m_scene.clear();
   // m_roadmap.drawVoronoiDiagram(m_scene, true);
	// Build depending on the build state
	/* 
	switch(m_state){
	case BUILD_STATE_MAP_OPEN:
		m_roadmap.buildHexgaonLattice();
		m_pOverlayLatticAction->setEnabled(false);
	case BUILD_STATE_LATTICE_OVERLAYED: 
        std::cout<<"|||||||||||||||||||||||||||||||go tp removeExcessEdges"<<std::endl;
		m_roadmap.removeExcessEdges(doConnection);
		m_pLocateBoundaryAction->setEnabled(false);
	default:;
	}
	*/    
	// Draw environment

	//drawBasicEnvironment();

	// Draw the lattice (now truncated)
	/*  
	m_roadmap.drawHexagonLattice(m_scene, true);
#ifdef _DEBUG
	m_roadmap.drawBoundingCycle(m_scene);
	//m_roadmap.drawJointBoundingCycle(m_scene);
    m_roadmap.drawVertexIds(m_scene);
#endif

	// Set build state
	m_state = BUILD_STATE_TRIMMED;   

	// Enable random instance creation
	m_pCreateAction->setEnabled(true); 
    if(m_pBoundingPolyAGI2 != 0){
        m_pView->setSceneRect(m_pBoundingPolyAGI2->boundingRect());
        m_pView->fitInView(m_boundingRect, Qt::KeepAspectRatio);
    }
	*/
}

// Create and solve a random problem over current graph
/* 
void MainWindow::createRandomProblem() {
	
	
	m_numRobots = 1;
	// Clean up
	m_scene.clear();
	m_boundingPoly.clear();
	m_boundingPoly2.clear();
	m_envPolyList.clear();
	m_envObsPolyList.clear();
	m_PolyAGIList.clear();

	// Reading in the environment, first the raidus of the (disc) robot
	//std::ifstream ifs(qPrintable(m_fileNameq));
	double radius = 30;
	//ifs >> radius;
	int numberOfPolygons = m_pLineEdit->text().toInt();
	double spacing = m_pMinDistLineEdit->text().toDouble();
	// Then the number of obstacle polygons


	// Then read in all obstacle polygons and compute the configuration space for a single disc
	double center_x, center_y, yaw;
	double x_max, x_min, y_max, y_min, yaw_max, yaw_min;
	yaw_max = 1.57;
	yaw_min = -1.57;
	x_max = 1800;
	x_min = 150;
	y_min = 150;
	y_max = 1700;
	std::vector<Polygon_2> created_polys;
	bool is_valid = true;
	bool is_start_valid = true;
	while(1){
		srand(time(NULL));
		is_start_valid = true;
		is_valid = true;
		created_polys.clear();
	for (int i = 0; i < numberOfPolygons; i++) {
		// Load polygon
		is_valid = true;

		Polygon_2 tp;

		//Point_2 firstOne_1;
		int trial_times = 0;
		do {
			center_x = (rand() % 10000) / 10000.*(x_max - x_min) + x_min;
			center_y = (rand() % 10000) / 10000.*(y_max - y_min) + y_min;
			yaw = (rand() % 10000) / 10000.*(yaw_max - yaw_min) + yaw_min;
			tp = generatePoly(center_x, center_y, yaw);

			for (int j = 0; j < created_polys.size(); j++) {
				if (bg::intersects(tp, created_polys[j])) {
					is_valid = false;
					break;
				}
			}
			trial_times++;
			if (trial_times > 1000) {
				std::cout << "failed to find" << std::endl;
				is_start_valid = false;
				break;
			}
		} while (!is_valid);
		if (!is_start_valid) {
			break;
		}
		// bg::append(tp.outer(), firstOne_1);
		m_envPolyList.push_back(tp);
		created_polys.push_back(tp);

		// Add raw obstacle to scene and set fill to be true with fill transparency
		AdvancedGraphicsItem<Polygon_2>* pAGI = new AdvancedGraphicsItem<Polygon_2>(&(m_envPolyList.back()));

		// m_scene.addItem(pAGI);
		pAGI->m_bShowEdge = true;
		pAGI->m_bShowVertices = true;
		pAGI->m_bFill = false;
		pAGI->m_fillBrush = QColor(16, 16, 16, 192);
		m_PolyAGIList.push_back(pAGI);

		// Computing the Minkowski sum
		int split_num = 8;
		Polygon_2 ep = growPolygonByRadius(tp, radius, split_num);
		bg::correct(ep);
		//split_num = 4;
		//Polygon_2 vp = growPolygonByRadius(tp, radius, split_num);
		//bg::correct(vp);

		m_envPolyVoronoiList.push_back(ep);
		m_envObsPolyList.push_back(ep);

		// Add the configuration space obstacle to the scene
		pAGI = new AdvancedGraphicsItem<Polygon_2>(&(m_envObsPolyList.back()));

		// m_scene.addItem(pAGI);
		pAGI->m_bFill = true;
		pAGI->m_edgePen = QPen(Qt::gray, 0.25, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
		pAGI->m_vertexPen = QPen(Qt::black, 0.5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
		m_PolyAGIList.push_back(pAGI);
	}
	if (is_valid) {
		break;
	}
}
	// Then read the bounding rectangle (configuration space)
	int numberOfBoundingVertex;
	Point_2 p;
	p.set<0>(0);
	p.set<1>(0);
	bg::append(m_boundingPoly.outer(), p);
	p.set<0>(0);
	p.set<1>(2000);
	bg::append(m_boundingPoly.outer(), p);
	p.set<0>(2000);
	p.set<1>(2000);
	bg::append(m_boundingPoly.outer(), p);
	p.set<0>(2000);
	p.set<1>(0);
	bg::append(m_boundingPoly.outer(), p);
	bg::correct(m_boundingPoly);

	m_envPolyVoronoiList.push_back(m_boundingPoly);
	// Add to scence

	m_pBoundingPolyAGI = new AdvancedGraphicsItem<Polygon_2>(&m_boundingPoly);
	m_pBoundingPolyAGI->m_bFill = false;
	m_pBoundingPolyAGI->m_bShowVertices = false;
	m_pBoundingPolyAGI->m_bShowEdge = true;
	m_pBoundingPolyAGI->m_edgePen = QPen(Qt::gray, 0.25, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
	m_PolyAGIList.push_back(m_pBoundingPolyAGI);
	m_scene.addItem(m_pBoundingPolyAGI);

	// Then compute the outside bounding rectangle
	double x1, y1, x2, y2;
	std::vector<Point_2> const& points = m_boundingPoly.outer();

	x1 = points[0].get<0>(); y1 = points[0].get<1>();
	x2 = points[2].get<0>(); y2 = points[2].get<1>();
	bg::append(m_boundingPoly2.outer(), Point_2(x1 - radius, y1 - radius));
	bg::append(m_boundingPoly2.outer(), Point_2(x1 - radius, y2 + radius));
	bg::append(m_boundingPoly2.outer(), Point_2(x2 + radius, y2 + radius));
	bg::append(m_boundingPoly2.outer(), Point_2(x2 + radius, y1 - radius));
	//	bg::append(m_boundingPoly2.outer(), Point_2(x1 - radius, y1 - radius));
	bg::correct(m_boundingPoly2);
	// Add to scene
	m_pBoundingPolyAGI2 = new AdvancedGraphicsItem<Polygon_2>(&m_boundingPoly2);
	m_pBoundingPolyAGI2->m_bFill = false;
	m_pBoundingPolyAGI2->m_bShowVertices = false;
	m_pBoundingPolyAGI2->m_bShowEdge = true;
	m_pBoundingPolyAGI2->m_edgePen = QPen(Qt::black, 0.25, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
	m_PolyAGIList.push_back(m_pBoundingPolyAGI2);
	// m_scene.addItem(m_pBoundingPolyAGI2);
	std::ofstream ofs("C:/Users/wei/Desktop/optimal_robot_picking/trunk/optimal_robot_picking/test-envs/test.txt", std::ofstream::out);
	ofs << 30 << std::endl;
	ofs << numberOfPolygons<< std::endl;
	for (int q = 0; q < created_polys.size(); q++) {
		ofs << created_polys[q].outer().size()-1 << std::endl;
		for (int v = 0; v < created_polys[q].outer().size()-1; v++) {
			ofs << created_polys[q].outer()[v].get<0>() <<" "<< created_polys[q].outer()[v].get<1>()<<std::endl;
		}
	}
	ofs << 4 <<std::endl;
	ofs << 0 <<" "<< 0 <<std::endl;
	ofs << 0 <<" "<< 2000 << std::endl;
	ofs << 2000 <<" "<< 2000<<std::endl;
	ofs << 2000 <<" " << 0 << std::endl;
	ofs.close();

	m_radius = radius;

	drawBasicEnvironment();

	// Do roadmap building setup
	m_roadmap.buildRoadmap(&m_envObsPolyList, &m_envPolyList, &m_boundingPoly, &m_envPolyVoronoiList, radius, m_scene);
	// m_roadmap.addToScene(m_scene);

	m_boundingRect = QRectF(x1 - radius * 4, y1 - radius * 4, x2 + radius * 4, y2 + radius * 4);

	// Fit to the view
	fitView();

	// Enable the actions for building the graph
	m_state = BUILD_STATE_MAP_OPEN;
	m_pOverlayLatticAction->setEnabled(true);
	m_pLocateBoundaryAction->setEnabled(true);
	//m_pCreateAction->setEnabled(false);
	m_pSolveAction->setEnabled(true);
	m_pPlayAction->setEnabled(false);

}
*/
void MainWindow::addIntoEnv(Polygon_2 tp, int index) {
	m_envPolyList.push_back(tp);
	double radius = 30;
	Point_2 down_left, upper_right;
	// Add raw obstacle to scene and set fill to be true with fill transparency
	AdvancedGraphicsItem<Polygon_2>* pAGI = new AdvancedGraphicsItem<Polygon_2>(&(m_envPolyList.back()));
	double x, y;
	down_left = tp.outer()[0];
	upper_right = tp.outer()[2];
	x = (down_left.get<0>() + upper_right.get<0>()) / 2;
	y = (down_left.get<1>() + upper_right.get<1>()) / 2;
	// m_scene.addItem(pAGI);
	pAGI->m_bShowEdge = true;
	pAGI->m_bShowVertices = true;
	pAGI->m_bFill = false;
	pAGI->m_fillBrush = QColor(120, 120, 120, 192);
	m_PolyAGIList.push_back(pAGI);

	// Computing the Minkowski sum
	int split_num = 8;
	Polygon_2 ep = growPolygonByRadius(tp, radius, split_num);
	bg::correct(ep);
	//split_num = 4;
	//Polygon_2 vp = growPolygonByRadius(tp, radius, split_num);
	//bg::correct(vp);

	m_envPolyVoronoiList.push_back(ep);
	m_envObsPolyList.push_back(ep);

	// Add the configuration space obstacle to the scene
	pAGI = new AdvancedGraphicsItem<Polygon_2>(&(m_envObsPolyList.back()));

	//m_scene.addItem(pAGI);
	pAGI->m_bFill = true;
	pAGI->m_edgePen = QPen(Qt::gray, 0.25, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
	pAGI->m_vertexPen = QPen(Qt::black, 0.5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
	m_PolyAGIList.push_back(pAGI);

	QFont font;
	font.setPointSizeF(15);
	font.setBold(false);
	QGraphicsSimpleTextItem *ti = m_scene.addSimpleText(QString::number(index), font);
	ti->setPos(x, y);
	ti->setPen(QPen(QColor(Qt::white), 0.5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
	ti->setZValue(1);
}

bool MainWindow::isTwoPolyTooClose(Polygon_2 a, Polygon_2 b) {
	double a_x, a_y, b_x, b_y;
	a_x = (a.outer()[0].get<0>() + a.outer()[1].get<0>() + a.outer()[2].get<0>() + a.outer()[3].get<0>()) / 4.0;
	a_y = (a.outer()[0].get<1>() + a.outer()[1].get<1>() + a.outer()[2].get<1>() + a.outer()[3].get<1>()) / 4.0;
	b_x = (b.outer()[0].get<0>() + b.outer()[1].get<0>() + b.outer()[2].get<0>() + b.outer()[3].get<0>()) / 4.0;
	b_y = (b.outer()[0].get<1>() + b.outer()[1].get<1>() + b.outer()[2].get<1>() + b.outer()[3].get<1>()) / 4.0;
	if (sqrt(pow(a_x - b_x, 2) + pow(a_y - b_y, 2)) < 70) {
		return true;
	}
	else {
		return false;
	}

}

Polygon_2 MainWindow::addNewNearTetris(std::vector<Polygon_2> exist_polys, std::vector<Polygon_2> all_created_polys) {
	double yaw;
	double length = 145;
	double obj_between_dist = 15;
	//srand(time(NULL));
	double width = 30;
	Polygon_2 new_poly;
	std::uniform_int_distribution<int> distribution(0, 10000);
	std::random_device rd;
	std::mt19937 engine(rd());
	int type = 0;
#ifdef ALL_FLAT
	yaw = 0;
#else
	if (distribution(engine) / 10000. < 0.5) {
		yaw = 1.57;
	}
	else {
		yaw = 0;
	}
#endif
	double type_double = distribution(engine) / 10000.;
	if (type_double < 0.3333) {
		type = 0;
	}
	else if (type_double < 0.6667 && type_double > 0.3333) {
		type = 1;
	}
	else {
		type = 2;
	}
#ifdef NOT_AXIS_ALIGNED
	yaw = distribution(engine) / 10000. * 3.14;
#endif
	Segment_2 border_1, border_2, border_3, border_4;
#ifdef BIG_ENV
	border_1.first = Point_2(0, 0);
	border_1.second = Point_2(5000, 0);
	border_2.first = Point_2(5000, 0);
	border_2.second = Point_2(5000, 5000);
	border_3.first = Point_2(5000, 5000);
	border_3.second = Point_2(0, 5000);
	border_4.first = Point_2(0, 5000);
	border_4.second = Point_2(0, 0);

#endif

#ifdef SMALL_ENV
	border_1.first = Point_2(0, 0);
	border_1.second = Point_2(1000, 0);
	border_2.first = Point_2(1000, 0);
	border_2.second = Point_2(1000, 1000);
	border_3.first = Point_2(1000, 1000);
	border_3.second = Point_2(0, 1000);
	border_4.first = Point_2(0, 1000);
	border_4.second = Point_2(0, 0);
#endif
	std::vector<Polygon_2> final_outer;
	if (exist_polys.size() == 1) {

		Point_2 upper_left, upper_right, down_left, down_right;
		Polygon_2 outer_poly;
		// upper_left = exist_polys[0].outer()[1];
		// upper_right = exist_polys[0].outer()[2];
		// down_right = exist_polys[0].outer()[3];
		// down_left = exist_polys[0].outer()[0];
		getBoundingPoly(exist_polys[0], upper_left, upper_right, down_left, down_right, obj_between_dist);
		// if (yaw > 0) {
		// 	upper_left.set<0>(upper_left.get<0>() - obj_between_dist - width / 2);
		// 	upper_left.set<1>(upper_left.get<1>() - obj_between_dist - length / 2);
		// 	upper_right.set<0>(upper_right.get<0>() + obj_between_dist + width / 2);
		// 	upper_right.set<1>(upper_right.get<1>() - obj_between_dist - length / 2);
		// 	down_right.set<0>(down_right.get<0>() + obj_between_dist + width / 2);
		// 	down_right.set<1>(down_right.get<1>() + obj_between_dist + length / 2);
		// 	down_left.set<0>(down_left.get<0>() - obj_between_dist - width / 2);
		// 	down_left.set<1>(down_left.get<1>() + obj_between_dist + length / 2);
		// }
		// else if (yaw == 0) {
		// 	upper_left.set<0>(upper_left.get<0>() - obj_between_dist - length / 2);
		// 	upper_left.set<1>(upper_left.get<1>() - obj_between_dist - width / 2);
		// 	upper_right.set<0>(upper_right.get<0>() + obj_between_dist + length / 2);
		// 	upper_right.set<1>(upper_right.get<1>() - obj_between_dist - width / 2);
		// 	down_right.set<0>(down_right.get<0>() + obj_between_dist + length / 2);
		// 	down_right.set<1>(down_right.get<1>() + obj_between_dist + width / 2);
		// 	down_left.set<0>(down_left.get<0>() - obj_between_dist - length / 2);
		// 	down_left.set<1>(down_left.get<1>() + obj_between_dist + width / 2);
		// }
		bg::append(outer_poly.outer(), upper_left);
		bg::append(outer_poly.outer(), upper_right);
		bg::append(outer_poly.outer(), down_right);
		bg::append(outer_poly.outer(), down_left);
		bg::correct(outer_poly);

		bool is_valid = false;
		while (!is_valid) {
			new_poly.outer().clear();
			is_valid = true;
			double rotation = distribution(engine) / 10000. *(3.14 + 3.14) - 3.14;
			Point_2 origin;
			origin.set<0>((upper_left.get<0>() + upper_right.get<0>()) / 2);
			origin.set<1>((upper_left.get<1>() + down_left.get<1>()) / 2);
			Point_2 far_pt;
			far_pt.set<0>(500 * cos(rotation) + origin.get<0>());
			far_pt.set<1>(500 * sin(rotation) + origin.get<1>());
			Linestring_2 inter;
			bg::append(inter, origin);
			bg::append(inter, far_pt);
			std::vector<Point_2> inter_pt_list;
			bg::intersection(outer_poly, inter, inter_pt_list);
			if (inter_pt_list.size() == 1) {
				Point_2 inter_pt = inter_pt_list[0];
#ifdef DIFFERENT_SIZE
				double different_length = (distribution(engine) / 10000. * 2.5 + 1) * OBJ_LENGTH;
				double different_width = OBJ_WIDTH;
				generatePoly(new_poly, inter_pt.get<0>(), inter_pt.get<1>(), yaw, different_length, different_width);

#else
				generateTetrisBlock(new_poly, inter_pt.get<0>(), inter_pt.get<1>(), yaw, type);
#endif
				// if (yaw > 0) {
				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() - width / 2, inter_pt.get<1>() + length / 2));
				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() - width / 2, inter_pt.get<1>() - length / 2));
				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() + width / 2, inter_pt.get<1>() - length / 2));
				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() + width / 2, inter_pt.get<1>() + length / 2));

				// 	bg::correct(new_poly);
				// }
				// else if (yaw == 0) {
				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() - length / 2, inter_pt.get<1>() + width / 2));
				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() - length / 2, inter_pt.get<1>() - width / 2));
				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() + length / 2, inter_pt.get<1>() - width / 2));
				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() + length / 2, inter_pt.get<1>() + width / 2));

				// 	bg::correct(new_poly);
				// }
				if (bg::intersects(new_poly, exist_polys[0])) {
					is_valid = false;
				}
				if (bg::intersects(new_poly, border_1) || bg::intersects(new_poly, border_2) || bg::intersects(new_poly, border_3) || bg::intersects(new_poly, border_4)) {
					is_valid = false;
				}
				for (int p = 0; p < all_created_polys.size(); p++) {
					if (bg::intersects(all_created_polys[p], new_poly)) {
						is_valid = false;
						break;
					}
				}
#ifdef SMALL_ENV
				if (bg::distance(new_poly, Point_2(500, 1000)) < 40) {
					is_valid = false;
				}
#endif
#ifdef BIG_ENV
				if (bg::distance(new_poly, Point_2(2500, 5000)) < 40) {
					is_valid = false;
				}
#endif
#ifdef BIG_ENV
				if (new_poly.outer()[0].get<1>() > 5000 || new_poly.outer()[0].get<1>() < 0 || new_poly.outer()[0].get<0>() > 5000 || new_poly.outer()[0].get<0>() < 0) {
					is_valid = false;
				}
#endif
#ifdef SMALL_ENV
				if (new_poly.outer()[0].get<1>() > 1000 || new_poly.outer()[0].get<1>() < 0 || new_poly.outer()[0].get<0>() > 1000 || new_poly.outer()[0].get<0>() < 0) {
					is_valid = false;
				}
#endif
			}
			else {
				std::cout << "error: intersection output more than 1 pts" << std::endl;
			}
		}
	}
	else if (exist_polys.size() == 2) {
		Point_2 upper_left, upper_right, down_left, down_right;
		Polygon_2 outer_poly_1, outer_poly_2;

		getBoundingPoly(exist_polys[0], upper_left, upper_right, down_left, down_right, obj_between_dist);

		// upper_left = exist_polys[0].outer()[1];
		// upper_right = exist_polys[0].outer()[2];
		// down_right = exist_polys[0].outer()[3];
		// down_left = exist_polys[0].outer()[0];
		// if (yaw > 0) {
		// 	upper_left.set<0>(upper_left.get<0>() - obj_between_dist - width / 2);
		// 	upper_left.set<1>(upper_left.get<1>() - obj_between_dist - length / 2);
		// 	upper_right.set<0>(upper_right.get<0>() + obj_between_dist + width / 2);
		// 	upper_right.set<1>(upper_right.get<1>() - obj_between_dist - length / 2);
		// 	down_right.set<0>(down_right.get<0>() + obj_between_dist + width / 2);
		// 	down_right.set<1>(down_right.get<1>() + obj_between_dist + length / 2);
		// 	down_left.set<0>(down_left.get<0>() - obj_between_dist - width / 2);
		// 	down_left.set<1>(down_left.get<1>() + obj_between_dist + length / 2);
		// }
		// else if (yaw == 0) {
		// 	upper_left.set<0>(upper_left.get<0>() - obj_between_dist - length / 2);
		// 	upper_left.set<1>(upper_left.get<1>() - obj_between_dist - width / 2);
		// 	upper_right.set<0>(upper_right.get<0>() + obj_between_dist + length / 2);
		// 	upper_right.set<1>(upper_right.get<1>() - obj_between_dist - width / 2);
		// 	down_right.set<0>(down_right.get<0>() + obj_between_dist + length / 2);
		// 	down_right.set<1>(down_right.get<1>() + obj_between_dist + width / 2);
		// 	down_left.set<0>(down_left.get<0>() - obj_between_dist - length / 2);
		// 	down_left.set<1>(down_left.get<1>() + obj_between_dist + width / 2);
		// }
		bg::append(outer_poly_1.outer(), upper_left);
		bg::append(outer_poly_1.outer(), upper_right);
		bg::append(outer_poly_1.outer(), down_right);
		bg::append(outer_poly_1.outer(), down_left);
		bg::correct(outer_poly_1);


		getBoundingPoly(exist_polys[1], upper_left, upper_right, down_left, down_right, obj_between_dist);

		// upper_left = exist_polys[1].outer()[1];
		// upper_right = exist_polys[1].outer()[2];
		// down_right = exist_polys[1].outer()[3];
		// down_left = exist_polys[1].outer()[0];
		// if (yaw > 0) {
		// 	upper_left.set<0>(upper_left.get<0>() - obj_between_dist - width / 2);
		// 	upper_left.set<1>(upper_left.get<1>() - obj_between_dist - length / 2);
		// 	upper_right.set<0>(upper_right.get<0>() + obj_between_dist + width / 2);
		// 	upper_right.set<1>(upper_right.get<1>() - obj_between_dist - length / 2);
		// 	down_right.set<0>(down_right.get<0>() + obj_between_dist + width / 2);
		// 	down_right.set<1>(down_right.get<1>() + obj_between_dist + length / 2);
		// 	down_left.set<0>(down_left.get<0>() - obj_between_dist - width / 2);
		// 	down_left.set<1>(down_left.get<1>() + obj_between_dist + length / 2);
		// }
		// else if (yaw == 0) {
		// 	upper_left.set<0>(upper_left.get<0>() - obj_between_dist - length / 2);
		// 	upper_left.set<1>(upper_left.get<1>() - obj_between_dist - width / 2);
		// 	upper_right.set<0>(upper_right.get<0>() + obj_between_dist + length / 2);
		// 	upper_right.set<1>(upper_right.get<1>() - obj_between_dist - width / 2);
		// 	down_right.set<0>(down_right.get<0>() + obj_between_dist + length / 2);
		// 	down_right.set<1>(down_right.get<1>() + obj_between_dist + width / 2);
		// 	down_left.set<0>(down_left.get<0>() - obj_between_dist - length / 2);
		// 	down_left.set<1>(down_left.get<1>() + obj_between_dist + width / 2);
		// }
		bg::append(outer_poly_2.outer(), upper_left);
		bg::append(outer_poly_2.outer(), upper_right);
		bg::append(outer_poly_2.outer(), down_right);
		bg::append(outer_poly_2.outer(), down_left);
		bg::correct(outer_poly_2);

		std::vector<Polygon_2> union_polys;
		bg::union_(outer_poly_1, outer_poly_2, union_polys);
		if (union_polys.size() == 1) {
			bool is_valid = false;
			while (!is_valid) {

				double rotation = distribution(engine) / 10000. *(3.14 + 3.14) - 3.14;
				Point_2 origin;
				origin.set<0>((upper_left.get<0>() + upper_right.get<0>()) / 2);
				origin.set<1>((upper_left.get<1>() + down_left.get<1>()) / 2);
				Point_2 far_pt;
				far_pt.set<0>(2500 * cos(rotation) + origin.get<0>());
				far_pt.set<1>(2500 * sin(rotation) + origin.get<1>());
				Linestring_2 inter;
				bg::append(inter, origin);
				bg::append(inter, far_pt);
				std::vector<Point_2> inter_pt_list;
				bg::intersection(union_polys[0], inter, inter_pt_list);
				is_valid = true;

				for (int k = 0; k < inter_pt_list.size(); k++) {
					Point_2 inter_pt = inter_pt_list[k];
					new_poly.outer().clear();
					// if (yaw > 0) {
					// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() - width / 2, inter_pt.get<1>() + length / 2));
					// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() - width / 2, inter_pt.get<1>() - length / 2));
					// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() + width / 2, inter_pt.get<1>() - length / 2));
					// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() + width / 2, inter_pt.get<1>() + length / 2));

					// 	bg::correct(new_poly);
					// }
					// else if (yaw == 0) {
					// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() - length / 2, inter_pt.get<1>() + width / 2));
					// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() - length / 2, inter_pt.get<1>() - width / 2));
					// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() + length / 2, inter_pt.get<1>() - width / 2));
					// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() + length / 2, inter_pt.get<1>() + width / 2));

					// 	bg::correct(new_poly);
					// }
#ifdef DIFFERENT_SIZE
					double different_length = (distribution(engine) / 10000. * 2.5 + 1) * OBJ_LENGTH;
					double different_width = OBJ_WIDTH;
					generatePoly(new_poly, inter_pt.get<0>(), inter_pt.get<1>(), yaw, different_length, different_width);

#else
					generateTetrisBlock(new_poly, inter_pt.get<0>(), inter_pt.get<1>(), yaw, type);

#endif

					if (bg::intersects(new_poly, border_1) || bg::intersects(new_poly, border_2) || bg::intersects(new_poly, border_3) || bg::intersects(new_poly, border_4)) {
						is_valid = false;
					}
					for (int t = 0; t < exist_polys.size(); t++) {
						if (bg::intersects(exist_polys[t], new_poly)) {
							is_valid = false;
							break;
						}
						if (isTwoPolyTooClose(exist_polys[t], new_poly)) {
							is_valid = false;
							break;
						}

					}
					for (int p = 0; p < all_created_polys.size(); p++) {
						if (bg::intersects(all_created_polys[p], new_poly)) {
							is_valid = false;
							break;
						}
					}
#ifdef SMALL_ENV
					if (bg::distance(new_poly, Point_2(500, 1000)) < 40) {
						is_valid = false;
					}
#endif
#ifdef BIG_ENV
					if (bg::distance(new_poly, Point_2(2500, 5000)) < 40) {
						is_valid = false;
					}
#endif
#ifdef BIG_ENV
					if (new_poly.outer()[0].get<1>() > 5000 || new_poly.outer()[0].get<1>() < 0 || new_poly.outer()[0].get<0>() > 5000 || new_poly.outer()[0].get<0>() < 0) {
						is_valid = false;
					}
#endif
#ifdef SMALL_ENV
					if (new_poly.outer()[0].get<1>() > 1000 || new_poly.outer()[0].get<1>() < 0 || new_poly.outer()[0].get<0>() > 1000 || new_poly.outer()[0].get<0>() < 0) {
						is_valid = false;
					}
#endif
					if (is_valid) {
						break;
					}
				}
			}





		}
		else {
			std::cout << "error(2 polys): union_ returns more than 1 polygons" << std::endl;
		}


	}
	else {
		Point_2 upper_left, upper_right, down_left, down_right;

		Polygon_2 outer_poly_1, outer_poly_2;
		Polygon_2 current_poly;
		std::vector<Polygon_2> outer_poly_vector;
		for (int k = 0; k < exist_polys.size(); k++) {
			current_poly.outer().clear();
			getBoundingPoly(exist_polys[k], upper_left, upper_right, down_left, down_right, obj_between_dist);

			// upper_left = exist_polys[k].outer()[1];
			// upper_right = exist_polys[k].outer()[2];
			// down_right = exist_polys[k].outer()[3];
			// down_left = exist_polys[k].outer()[0];
			// if (yaw > 0) {
			// 	upper_left.set<0>(upper_left.get<0>() - obj_between_dist - width / 2);
			// 	upper_left.set<1>(upper_left.get<1>() - obj_between_dist - length / 2);
			// 	upper_right.set<0>(upper_right.get<0>() + obj_between_dist + width / 2);
			// 	upper_right.set<1>(upper_right.get<1>() - obj_between_dist - length / 2);
			// 	down_right.set<0>(down_right.get<0>() + obj_between_dist + width / 2);
			// 	down_right.set<1>(down_right.get<1>() + obj_between_dist + length / 2);
			// 	down_left.set<0>(down_left.get<0>() - obj_between_dist - width / 2);
			// 	down_left.set<1>(down_left.get<1>() + obj_between_dist + length / 2);
			// }
			// else if (yaw == 0) {
			// 	upper_left.set<0>(upper_left.get<0>() - obj_between_dist - length / 2);
			// 	upper_left.set<1>(upper_left.get<1>() - obj_between_dist - width / 2);
			// 	upper_right.set<0>(upper_right.get<0>() + obj_between_dist + length / 2);
			// 	upper_right.set<1>(upper_right.get<1>() - obj_between_dist - width / 2);
			// 	down_right.set<0>(down_right.get<0>() + obj_between_dist + length / 2);
			// 	down_right.set<1>(down_right.get<1>() + obj_between_dist + width / 2);
			// 	down_left.set<0>(down_left.get<0>() - obj_between_dist - length / 2);
			// 	down_left.set<1>(down_left.get<1>() + obj_between_dist + width / 2);
			// }
			bg::append(current_poly.outer(), upper_left);
			bg::append(current_poly.outer(), upper_right);
			bg::append(current_poly.outer(), down_right);
			bg::append(current_poly.outer(), down_left);
			bg::correct(current_poly);
			outer_poly_vector.push_back(current_poly);
		}
		Polygon_2 first = outer_poly_vector.back();
		outer_poly_vector.pop_back();
		Polygon_2 second;
		std::vector<Polygon_2> union_polys;
		while (outer_poly_vector.size() > 0) {
			second = outer_poly_vector.back();
			bg::union_(first, second, union_polys);
			first = union_polys[0];
			outer_poly_vector.pop_back();
		}
		bool is_valid = false;
		while (!is_valid) {

			double rotation = distribution(engine) / 10000. *(3.14 + 3.14) - 3.14;

			//srand(rand()%10000);
			Point_2 origin;
			origin.set<0>((upper_left.get<0>() + upper_right.get<0>()) / 2);
			origin.set<1>((upper_left.get<1>() + down_left.get<1>()) / 2);
			Point_2 far_pt;
			far_pt.set<0>(2500 * cos(rotation) + origin.get<0>());
			far_pt.set<1>(2500 * sin(rotation) + origin.get<1>());
			Linestring_2 inter;
			bg::append(inter, origin);
			bg::append(inter, far_pt);
			std::vector<Point_2> inter_pt_list;
			bg::intersection(first, inter, inter_pt_list);

			for (int k = 0; k < inter_pt_list.size(); k++) {
				is_valid = true;
				Point_2 inter_pt = inter_pt_list[k];
				new_poly.outer().clear();
				// if (yaw > 0) {
				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() - width / 2, inter_pt.get<1>() + length / 2));
				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() - width / 2, inter_pt.get<1>() - length / 2));
				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() + width / 2, inter_pt.get<1>() - length / 2));
				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() + width / 2, inter_pt.get<1>() + length / 2));

				// 	bg::correct(new_poly);
				// }
				// else if (yaw == 0) {
				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() - length / 2, inter_pt.get<1>() + width / 2));
				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() - length / 2, inter_pt.get<1>() - width / 2));
				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() + length / 2, inter_pt.get<1>() - width / 2));
				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() + length / 2, inter_pt.get<1>() + width / 2));

				// 	bg::correct(new_poly);
				// }

#ifdef DIFFERENT_SIZE
				double different_length = (distribution(engine) / 10000. * 2.5 + 1) * OBJ_LENGTH;
				double different_width = OBJ_WIDTH;
				generatePoly(new_poly, inter_pt.get<0>(), inter_pt.get<1>(), yaw, different_length, different_width);

#else
				generateTetrisBlock(new_poly, inter_pt.get<0>(), inter_pt.get<1>(), yaw, type);

#endif
				if (bg::intersects(new_poly, border_1) || bg::intersects(new_poly, border_2) || bg::intersects(new_poly, border_3) || bg::intersects(new_poly, border_4)) {
					is_valid = false;
				}
				for (int t = 0; t < exist_polys.size(); t++) {
					if (bg::intersects(exist_polys[t], new_poly)) {
						is_valid = false;
						break;
					}
					if (isTwoPolyTooClose(exist_polys[t], new_poly)) {
						is_valid = false;
						break;
					}

				}
				for (int p = 0; p < all_created_polys.size(); p++) {
					if (bg::intersects(all_created_polys[p], new_poly)) {
						is_valid = false;
						break;
					}
				}
#ifdef SMALL_ENV
				if (bg::distance(new_poly, Point_2(500, 1000)) < 40) {
					is_valid = false;
				}
#endif
#ifdef BIG_ENV
				if (bg::distance(new_poly, Point_2(2500, 5000)) < 40) {
					is_valid = false;
				}
#endif
#ifdef BIG_ENV
				if (new_poly.outer()[0].get<1>() > 5000 || new_poly.outer()[0].get<1>() < 0 || new_poly.outer()[0].get<0>() > 5000 || new_poly.outer()[0].get<0>() < 0) {
					is_valid = false;
				}
#endif
#ifdef SMALL_ENV
				if (new_poly.outer()[0].get<1>() > 1000 || new_poly.outer()[0].get<1>() < 0 || new_poly.outer()[0].get<0>() > 1000 || new_poly.outer()[0].get<0>() < 0) {
					is_valid = false;
				}
#endif
				if (is_valid) {
					break;
				}
			}
		}


	}

	return new_poly;
}


Polygon_2 MainWindow::addNewNearPoly(std::vector<Polygon_2> exist_polys, std::vector<Polygon_2> all_created_polys) {
	double yaw;
	double length = 145;
	double obj_between_dist = 15;
	//srand(time(NULL));
	double width = 30;
	Polygon_2 new_poly;
	std::uniform_int_distribution<int> distribution(0, 10000);
	std::random_device rd;
	std::mt19937 engine(rd());
#ifdef ALL_FLAT
	yaw = 0;
#else
	if (distribution(engine) / 10000. < 0.5) {
		yaw = 1.57;
	}
	else {
		yaw = 0;
	}
#endif
#ifdef NOT_AXIS_ALIGNED
	yaw = distribution(engine) / 10000. * 3.14;
#endif
	Segment_2 border_1, border_2, border_3, border_4;
#ifdef BIG_ENV
	border_1.first = Point_2(0, 0);
	border_1.second = Point_2(5000, 0);
	border_2.first = Point_2(5000, 0);     
	border_2.second = Point_2(5000, 5000);
	border_3.first = Point_2(5000, 5000);
	border_3.second = Point_2(0, 5000);
	border_4.first = Point_2(0, 5000);
	border_4.second = Point_2(0, 0);

#endif

#ifdef SMALL_ENV
	border_1.first = Point_2(0, 0);
	border_1.second = Point_2(1000, 0);
	border_2.first = Point_2(1000, 0);
	border_2.second = Point_2(1000, 1000);
	border_3.first = Point_2(1000, 1000);
	border_3.second = Point_2(0, 1000);
	border_4.first = Point_2(0, 1000);
	border_4.second = Point_2(0, 0);
#endif
	std::vector<Polygon_2> final_outer;
	if (exist_polys.size() == 1) {
		
			Point_2 upper_left, upper_right, down_left, down_right;
			Polygon_2 outer_poly;
			// upper_left = exist_polys[0].outer()[1];
			// upper_right = exist_polys[0].outer()[2];
			// down_right = exist_polys[0].outer()[3];
			// down_left = exist_polys[0].outer()[0];
			getBoundingPoly(exist_polys[0], upper_left, upper_right, down_left, down_right, obj_between_dist);
			// if (yaw > 0) {
			// 	upper_left.set<0>(upper_left.get<0>() - obj_between_dist - width / 2);
			// 	upper_left.set<1>(upper_left.get<1>() - obj_between_dist - length / 2);
			// 	upper_right.set<0>(upper_right.get<0>() + obj_between_dist + width / 2);
			// 	upper_right.set<1>(upper_right.get<1>() - obj_between_dist - length / 2);
			// 	down_right.set<0>(down_right.get<0>() + obj_between_dist + width / 2);
			// 	down_right.set<1>(down_right.get<1>() + obj_between_dist + length / 2);
			// 	down_left.set<0>(down_left.get<0>() - obj_between_dist - width / 2);
			// 	down_left.set<1>(down_left.get<1>() + obj_between_dist + length / 2);
			// }
			// else if (yaw == 0) {
			// 	upper_left.set<0>(upper_left.get<0>() - obj_between_dist - length / 2);
			// 	upper_left.set<1>(upper_left.get<1>() - obj_between_dist - width / 2);
			// 	upper_right.set<0>(upper_right.get<0>() + obj_between_dist + length / 2);
			// 	upper_right.set<1>(upper_right.get<1>() - obj_between_dist - width / 2);
			// 	down_right.set<0>(down_right.get<0>() + obj_between_dist + length / 2);
			// 	down_right.set<1>(down_right.get<1>() + obj_between_dist + width / 2);
			// 	down_left.set<0>(down_left.get<0>() - obj_between_dist - length / 2);
			// 	down_left.set<1>(down_left.get<1>() + obj_between_dist + width / 2);
			// }
			bg::append(outer_poly.outer(), upper_left);
			bg::append(outer_poly.outer(), upper_right);
			bg::append(outer_poly.outer(), down_right);
			bg::append(outer_poly.outer(), down_left);
			bg::correct(outer_poly);

			bool is_valid = false;
			while (!is_valid) {
				new_poly.outer().clear();
				is_valid = true;
				double rotation = distribution(engine) / 10000. *(3.14 + 3.14) - 3.14;
				Point_2 origin;
				origin.set<0>((upper_left.get<0>() + upper_right.get<0>()) / 2);
				origin.set<1>((upper_left.get<1>() + down_left.get<1>()) / 2);
				Point_2 far_pt;
				far_pt.set<0>(500 * cos(rotation) + origin.get<0>());
				far_pt.set<1>(500 * sin(rotation) + origin.get<1>());
				Linestring_2 inter;
				bg::append(inter, origin);
				bg::append(inter, far_pt);
				std::vector<Point_2> inter_pt_list;
				bg::intersection(outer_poly, inter, inter_pt_list);
				if (inter_pt_list.size() == 1) {
					Point_2 inter_pt = inter_pt_list[0];
					#ifdef DIFFERENT_SIZE
						double different_length = (distribution(engine) / 10000. * 2.5 + 1) * OBJ_LENGTH;
						double different_width = OBJ_WIDTH;
						generatePoly(new_poly, inter_pt.get<0>(), inter_pt.get<1>(), yaw, different_length, different_width);

					#else
						generatePoly(new_poly, inter_pt.get<0>(), inter_pt.get<1>(), yaw);
					#endif
					// if (yaw > 0) {
					// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() - width / 2, inter_pt.get<1>() + length / 2));
					// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() - width / 2, inter_pt.get<1>() - length / 2));
					// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() + width / 2, inter_pt.get<1>() - length / 2));
					// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() + width / 2, inter_pt.get<1>() + length / 2));

					// 	bg::correct(new_poly);
					// }
					// else if (yaw == 0) {
					// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() - length / 2, inter_pt.get<1>() + width / 2));
					// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() - length / 2, inter_pt.get<1>() - width / 2));
					// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() + length / 2, inter_pt.get<1>() - width / 2));
					// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() + length / 2, inter_pt.get<1>() + width / 2));

					// 	bg::correct(new_poly);
					// }
					if (bg::intersects(new_poly, exist_polys[0])) {
						is_valid = false;
					}
					if (bg::intersects(new_poly, border_1) || bg::intersects(new_poly, border_2) || bg::intersects(new_poly, border_3) || bg::intersects(new_poly, border_4)) {
						is_valid = false;
					}
					for (int p = 0; p < all_created_polys.size(); p++) {
						if (bg::intersects(all_created_polys[p], new_poly)) {
							is_valid = false;
							break;
						}
					}
#ifdef SMALL_ENV
					if (bg::distance(new_poly, Point_2(500, 1000)) < 40) {
						is_valid = false;
					}
#endif
#ifdef BIG_ENV
					if (bg::distance(new_poly, Point_2(2500, 5000)) < 40) {
						is_valid = false;
					}
#endif
#ifdef BIG_ENV
					if (new_poly.outer()[0].get<1>() > 5000 || new_poly.outer()[0].get<1>() < 0 || new_poly.outer()[0].get<0>() > 5000 || new_poly.outer()[0].get<0>() < 0) {
						is_valid = false;
					}
#endif
#ifdef SMALL_ENV
					if (new_poly.outer()[0].get<1>() > 1000 || new_poly.outer()[0].get<1>() < 0 || new_poly.outer()[0].get<0>() > 1000 || new_poly.outer()[0].get<0>() < 0) {
						is_valid = false;
					}
#endif
				}
				else {
					std::cout << "error: intersection output more than 1 pts" << std::endl;
				}
			}
	}
	else if (exist_polys.size() == 2) {
		Point_2 upper_left, upper_right, down_left, down_right;
		Polygon_2 outer_poly_1, outer_poly_2;

		getBoundingPoly(exist_polys[0], upper_left, upper_right, down_left, down_right, obj_between_dist);

		// upper_left = exist_polys[0].outer()[1];
		// upper_right = exist_polys[0].outer()[2];
		// down_right = exist_polys[0].outer()[3];
		// down_left = exist_polys[0].outer()[0];
		// if (yaw > 0) {
		// 	upper_left.set<0>(upper_left.get<0>() - obj_between_dist - width / 2);
		// 	upper_left.set<1>(upper_left.get<1>() - obj_between_dist - length / 2);
		// 	upper_right.set<0>(upper_right.get<0>() + obj_between_dist + width / 2);
		// 	upper_right.set<1>(upper_right.get<1>() - obj_between_dist - length / 2);
		// 	down_right.set<0>(down_right.get<0>() + obj_between_dist + width / 2);
		// 	down_right.set<1>(down_right.get<1>() + obj_between_dist + length / 2);
		// 	down_left.set<0>(down_left.get<0>() - obj_between_dist - width / 2);
		// 	down_left.set<1>(down_left.get<1>() + obj_between_dist + length / 2);
		// }
		// else if (yaw == 0) {
		// 	upper_left.set<0>(upper_left.get<0>() - obj_between_dist - length / 2);
		// 	upper_left.set<1>(upper_left.get<1>() - obj_between_dist - width / 2);
		// 	upper_right.set<0>(upper_right.get<0>() + obj_between_dist + length / 2);
		// 	upper_right.set<1>(upper_right.get<1>() - obj_between_dist - width / 2);
		// 	down_right.set<0>(down_right.get<0>() + obj_between_dist + length / 2);
		// 	down_right.set<1>(down_right.get<1>() + obj_between_dist + width / 2);
		// 	down_left.set<0>(down_left.get<0>() - obj_between_dist - length / 2);
		// 	down_left.set<1>(down_left.get<1>() + obj_between_dist + width / 2);
		// }
		bg::append(outer_poly_1.outer(), upper_left);
		bg::append(outer_poly_1.outer(), upper_right);
		bg::append(outer_poly_1.outer(), down_right);
		bg::append(outer_poly_1.outer(), down_left);
		bg::correct(outer_poly_1);


		getBoundingPoly(exist_polys[1], upper_left, upper_right, down_left, down_right, obj_between_dist);

		// upper_left = exist_polys[1].outer()[1];
		// upper_right = exist_polys[1].outer()[2];
		// down_right = exist_polys[1].outer()[3];
		// down_left = exist_polys[1].outer()[0];
		// if (yaw > 0) {
		// 	upper_left.set<0>(upper_left.get<0>() - obj_between_dist - width / 2);
		// 	upper_left.set<1>(upper_left.get<1>() - obj_between_dist - length / 2);
		// 	upper_right.set<0>(upper_right.get<0>() + obj_between_dist + width / 2);
		// 	upper_right.set<1>(upper_right.get<1>() - obj_between_dist - length / 2);
		// 	down_right.set<0>(down_right.get<0>() + obj_between_dist + width / 2);
		// 	down_right.set<1>(down_right.get<1>() + obj_between_dist + length / 2);
		// 	down_left.set<0>(down_left.get<0>() - obj_between_dist - width / 2);
		// 	down_left.set<1>(down_left.get<1>() + obj_between_dist + length / 2);
		// }
		// else if (yaw == 0) {
		// 	upper_left.set<0>(upper_left.get<0>() - obj_between_dist - length / 2);
		// 	upper_left.set<1>(upper_left.get<1>() - obj_between_dist - width / 2);
		// 	upper_right.set<0>(upper_right.get<0>() + obj_between_dist + length / 2);
		// 	upper_right.set<1>(upper_right.get<1>() - obj_between_dist - width / 2);
		// 	down_right.set<0>(down_right.get<0>() + obj_between_dist + length / 2);
		// 	down_right.set<1>(down_right.get<1>() + obj_between_dist + width / 2);
		// 	down_left.set<0>(down_left.get<0>() - obj_between_dist - length / 2);
		// 	down_left.set<1>(down_left.get<1>() + obj_between_dist + width / 2);
		// }
		bg::append(outer_poly_2.outer(), upper_left);
		bg::append(outer_poly_2.outer(), upper_right);
		bg::append(outer_poly_2.outer(), down_right);
		bg::append(outer_poly_2.outer(), down_left);
		bg::correct(outer_poly_2);

		std::vector<Polygon_2> union_polys;
		bg::union_(outer_poly_1, outer_poly_2, union_polys);
		if (union_polys.size() == 1) {
			bool is_valid = false;
			while(!is_valid){
				
			double rotation = distribution(engine) / 10000. *(3.14 + 3.14) - 3.14;
			Point_2 origin;
			origin.set<0>((upper_left.get<0>() + upper_right.get<0>()) / 2);
			origin.set<1>((upper_left.get<1>() + down_left.get<1>()) / 2);
			Point_2 far_pt;
			far_pt.set<0>(2500 * cos(rotation) + origin.get<0>());
			far_pt.set<1>(2500 * sin(rotation) + origin.get<1>());
			Linestring_2 inter;
			bg::append(inter, origin);
			bg::append(inter, far_pt);
			std::vector<Point_2> inter_pt_list;
			bg::intersection(union_polys[0], inter, inter_pt_list);
			is_valid = true;

			for (int k = 0; k < inter_pt_list.size(); k++) {
				Point_2 inter_pt = inter_pt_list[k];
				new_poly.outer().clear();
				// if (yaw > 0) {
				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() - width / 2, inter_pt.get<1>() + length / 2));
				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() - width / 2, inter_pt.get<1>() - length / 2));
				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() + width / 2, inter_pt.get<1>() - length / 2));
				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() + width / 2, inter_pt.get<1>() + length / 2));

				// 	bg::correct(new_poly);
				// }
				// else if (yaw == 0) {
				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() - length / 2, inter_pt.get<1>() + width / 2));
				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() - length / 2, inter_pt.get<1>() - width / 2));
				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() + length / 2, inter_pt.get<1>() - width / 2));
				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() + length / 2, inter_pt.get<1>() + width / 2));

				// 	bg::correct(new_poly);
				// }
				#ifdef DIFFERENT_SIZE
					double different_length = (distribution(engine) / 10000. * 2.5 + 1) * OBJ_LENGTH;
					double different_width = OBJ_WIDTH;
					generatePoly(new_poly, inter_pt.get<0>(), inter_pt.get<1>(), yaw, different_length, different_width);

				#else
					generatePoly(new_poly, inter_pt.get<0>(), inter_pt.get<1>(), yaw);
				#endif

				if (bg::intersects(new_poly, border_1) || bg::intersects(new_poly, border_2) || bg::intersects(new_poly, border_3) || bg::intersects(new_poly, border_4)) {
					is_valid = false;
				}
				for (int t = 0; t < exist_polys.size(); t++) {
					if (bg::intersects(exist_polys[t], new_poly)) {
						is_valid = false;
						break;
					}
					if (isTwoPolyTooClose(exist_polys[t], new_poly)) {
						is_valid = false;
						break;
					}

				}
				for (int p = 0; p < all_created_polys.size(); p++) {
					if (bg::intersects(all_created_polys[p], new_poly)) {
						is_valid = false;
						break;
					}
				}
#ifdef SMALL_ENV
				if (bg::distance(new_poly, Point_2(500, 1000)) < 40) {
					is_valid = false;
				}
#endif
#ifdef BIG_ENV
				if (bg::distance(new_poly, Point_2(2500, 5000)) < 40) {
					is_valid = false;
				}
#endif
#ifdef BIG_ENV
				if (new_poly.outer()[0].get<1>() > 5000 || new_poly.outer()[0].get<1>() < 0 || new_poly.outer()[0].get<0>() > 5000 || new_poly.outer()[0].get<0>() < 0) {
					is_valid = false;
				}
#endif
#ifdef SMALL_ENV
				if (new_poly.outer()[0].get<1>() > 1000 || new_poly.outer()[0].get<1>() < 0 || new_poly.outer()[0].get<0>() > 1000 || new_poly.outer()[0].get<0>() < 0) {
					is_valid = false;
				}
#endif
				if (is_valid) {
					break;
				}
			}
			}
			
			
			
			
		
		}
		else {
			std::cout << "error(2 polys): union_ returns more than 1 polygons" << std::endl;
		}


	}
	else {
		Point_2 upper_left, upper_right, down_left, down_right;
		
		Polygon_2 outer_poly_1, outer_poly_2;
		Polygon_2 current_poly;
		std::vector<Polygon_2> outer_poly_vector;
		for (int k = 0; k < exist_polys.size(); k++) {
			current_poly.outer().clear();
			getBoundingPoly(exist_polys[k], upper_left, upper_right, down_left, down_right, obj_between_dist);

			// upper_left = exist_polys[k].outer()[1];
			// upper_right = exist_polys[k].outer()[2];
			// down_right = exist_polys[k].outer()[3];
			// down_left = exist_polys[k].outer()[0];
			// if (yaw > 0) {
			// 	upper_left.set<0>(upper_left.get<0>() - obj_between_dist - width / 2);
			// 	upper_left.set<1>(upper_left.get<1>() - obj_between_dist - length / 2);
			// 	upper_right.set<0>(upper_right.get<0>() + obj_between_dist + width / 2);
			// 	upper_right.set<1>(upper_right.get<1>() - obj_between_dist - length / 2);
			// 	down_right.set<0>(down_right.get<0>() + obj_between_dist + width / 2);
			// 	down_right.set<1>(down_right.get<1>() + obj_between_dist + length / 2);
			// 	down_left.set<0>(down_left.get<0>() - obj_between_dist - width / 2);
			// 	down_left.set<1>(down_left.get<1>() + obj_between_dist + length / 2);
			// }
			// else if (yaw == 0) {
			// 	upper_left.set<0>(upper_left.get<0>() - obj_between_dist - length / 2);
			// 	upper_left.set<1>(upper_left.get<1>() - obj_between_dist - width / 2);
			// 	upper_right.set<0>(upper_right.get<0>() + obj_between_dist + length / 2);
			// 	upper_right.set<1>(upper_right.get<1>() - obj_between_dist - width / 2);
			// 	down_right.set<0>(down_right.get<0>() + obj_between_dist + length / 2);
			// 	down_right.set<1>(down_right.get<1>() + obj_between_dist + width / 2);
			// 	down_left.set<0>(down_left.get<0>() - obj_between_dist - length / 2);
			// 	down_left.set<1>(down_left.get<1>() + obj_between_dist + width / 2);
			// }
			bg::append(current_poly.outer(), upper_left);
			bg::append(current_poly.outer(), upper_right);
			bg::append(current_poly.outer(), down_right);
			bg::append(current_poly.outer(), down_left);
			bg::correct(current_poly);
			outer_poly_vector.push_back(current_poly);
		}
		Polygon_2 first = outer_poly_vector.back();
		outer_poly_vector.pop_back();
		Polygon_2 second;
		std::vector<Polygon_2> union_polys;
		while (outer_poly_vector.size() > 0) {
			second = outer_poly_vector.back();
			bg::union_(first, second, union_polys);
			first = union_polys[0];
			outer_poly_vector.pop_back();
		}
		bool is_valid = false;
		while (!is_valid) {

			double rotation = distribution(engine) / 10000. *(3.14 + 3.14) - 3.14;
			
			//srand(rand()%10000);
			Point_2 origin;
			origin.set<0>((upper_left.get<0>() + upper_right.get<0>()) / 2);
			origin.set<1>((upper_left.get<1>() + down_left.get<1>()) / 2);
			Point_2 far_pt;
			far_pt.set<0>(2500 * cos(rotation) + origin.get<0>());
			far_pt.set<1>(2500 * sin(rotation) + origin.get<1>());
			Linestring_2 inter;
			bg::append(inter, origin);
			bg::append(inter, far_pt);
			std::vector<Point_2> inter_pt_list;
			bg::intersection(first, inter, inter_pt_list);

			for (int k = 0; k < inter_pt_list.size(); k++) {
				is_valid = true;
				Point_2 inter_pt = inter_pt_list[k];
				new_poly.outer().clear();
				// if (yaw > 0) {
				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() - width / 2, inter_pt.get<1>() + length / 2));
				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() - width / 2, inter_pt.get<1>() - length / 2));
				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() + width / 2, inter_pt.get<1>() - length / 2));
				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() + width / 2, inter_pt.get<1>() + length / 2));

				// 	bg::correct(new_poly);
				// }
				// else if (yaw == 0) {
				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() - length / 2, inter_pt.get<1>() + width / 2));
				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() - length / 2, inter_pt.get<1>() - width / 2));
				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() + length / 2, inter_pt.get<1>() - width / 2));
				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() + length / 2, inter_pt.get<1>() + width / 2));

				// 	bg::correct(new_poly);
				// }

				#ifdef DIFFERENT_SIZE
					double different_length = (distribution(engine) / 10000. * 2.5 + 1) * OBJ_LENGTH;
					double different_width = OBJ_WIDTH;
					generatePoly(new_poly, inter_pt.get<0>(), inter_pt.get<1>(), yaw, different_length, different_width);

				#else
					generatePoly(new_poly, inter_pt.get<0>(), inter_pt.get<1>(), yaw);
				#endif
				if (bg::intersects(new_poly, border_1) || bg::intersects(new_poly, border_2) || bg::intersects(new_poly, border_3) || bg::intersects(new_poly, border_4)) {
					is_valid = false;
				}
				for (int t = 0; t < exist_polys.size(); t++) {
					if (bg::intersects(exist_polys[t], new_poly)) {
						is_valid = false;
						break;
					}
					if (isTwoPolyTooClose(exist_polys[t], new_poly)) {
						is_valid = false;
						break;
					}

				}
				for (int p = 0; p < all_created_polys.size(); p++) {
					if (bg::intersects(all_created_polys[p], new_poly)) {
						is_valid = false;
						break;
					}
				}
#ifdef SMALL_ENV
				if (bg::distance(new_poly, Point_2(500, 1000)) < 40) {
					is_valid = false;
				}
#endif
#ifdef BIG_ENV
				if (bg::distance(new_poly, Point_2(2500, 5000)) < 40) {
					is_valid = false;
				}
#endif
#ifdef BIG_ENV
				if (new_poly.outer()[0].get<1>() > 5000 || new_poly.outer()[0].get<1>() < 0 || new_poly.outer()[0].get<0>() > 5000 || new_poly.outer()[0].get<0>() < 0) {
					is_valid = false;
				}
#endif
#ifdef SMALL_ENV
				if (new_poly.outer()[0].get<1>() > 1000 || new_poly.outer()[0].get<1>() < 0 || new_poly.outer()[0].get<0>() > 1000 || new_poly.outer()[0].get<0>() < 0) {
					is_valid = false;
				}
#endif
				if (is_valid) {
					break;
				}
			}
		}
		

	}

	return new_poly;
}

void MainWindow::createRandomProblem() {
	
#define REPEAT
	int current_case = 0;
#ifdef REPEAT  
	while (current_case < 10) {
#endif
	m_numRobots = 1;
	// Clean up
	m_scene.clear();
	m_boundingPoly.clear();
	m_boundingPoly2.clear();
	m_envPolyList.clear();
	m_envObsPolyList.clear();
	m_PolyAGIList.clear();
	int global_index = 0;
	std::uniform_int_distribution<int> distribution(0, 10000);
	std::random_device rd;
	std::mt19937 engine(rd());
	// Reading in the environment, first the raidus of the (disc) robot
	//std::ifstream ifs(qPrintable(m_fileNameq));
	double radius = 30;
	//ifs >> radius;
	int numberOfPolygons = m_pLineEdit->text().toInt();
	double spacing = m_pMinDistLineEdit->text().toDouble();
	// Then the number of obstacle polygons


	// Then read in all obstacle polygons and compute the configuration space for a single disc
	double center_x, center_y, yaw;
	double x_max, x_min, y_max, y_min, yaw_max, yaw_min;
	yaw_max = 1.57;
	yaw_min = -1.57;
#ifdef BIG_ENV
	if (numberOfPolygons == 5) {
		x_min = 1000;
		x_max = 1500;
		y_min = 1000;
		y_max = 1500;
	}
	else if (numberOfPolygons == 10) {
		x_min = 1000;
		x_max = 1500;
		y_min = 1000;
		y_max = 1500;
	}
	else if (numberOfPolygons == 15) {
		x_min = 600;
		x_max = 1900;
		y_min = 600;
		y_max = 1900;
	}
	else if (numberOfPolygons == 20) {
		x_min = 600;
		x_max = 1900;
		y_min = 600;
		y_max = 1900;
	}
	else if (numberOfPolygons == 25) {
		x_min = 500;
		x_max = 2000;
		y_min = 500;
		y_max = 2000;
	}
	else if (numberOfPolygons == 30) {
		x_min = 300;
		x_max = 2200;
		y_min = 300;
		y_max = 2200;
	}
	else if (numberOfPolygons == 35) {
		x_min = 200;
		x_max = 2300;
		y_min = 200;
		y_max = 2300;
	}
	else if (numberOfPolygons == 40) {
		x_min = 200;
		x_max = 2300;
		y_min = 200;
		y_max = 2300;
	}
	else if (numberOfPolygons == 45) {
		x_min = 100;
		x_max = 2400;
		y_min = 100;
		y_max = 2400;
	}
	else {
		x_min = 0;
		x_max = 2500;
		y_min = 0;
		y_max = 2500;
	}
#endif
#ifdef SMALL_ENV
	x_max = 900;
	x_min = 100;
	y_min = 100;
	y_max = 900;
#endif
	std::vector<Polygon_2> created_polys;
	bool is_valid = true;
	bool is_start_valid = true;
	
#ifdef MIXED_CLUSTER
	int each_cluster_num = 25;
	int cluster_num = 0;
	int total_obj_num = 0;
	std::vector<int> cluster_num_list;

	while (total_obj_num < numberOfPolygons) {
		int cluster_objs = floor(distribution(engine) / 10000. * (each_cluster_num - 1)) + 1;
		if (total_obj_num + cluster_objs > numberOfPolygons) {
			cluster_num_list.push_back(numberOfPolygons - total_obj_num);
			total_obj_num += (numberOfPolygons - total_obj_num);
		}
		else {
			cluster_num_list.push_back(cluster_objs);
			total_obj_num += cluster_objs;
		}
		cluster_num++;
		
	}
	
#else
	int each_cluster_num = 1;
	int cluster_num = numberOfPolygons / each_cluster_num;
	if (cluster_num * each_cluster_num < numberOfPolygons) {
		cluster_num++;
	}
	std::vector<int> cluster_num_list;
	for (int n = 0; n < cluster_num - 1; n++) {
		cluster_num_list.push_back(each_cluster_num);
	}
	cluster_num_list.push_back(numberOfPolygons - each_cluster_num*(cluster_num - 1));
#endif

	for (int i = 0; i < cluster_num; i++) {
		bool isValid = false;
		while (!isValid) {
			Polygon_2 tp;
			isValid = true;

			std::cout << "cluster " << i << std::endl;
			center_x = distribution(engine) / 10000.*(x_max - x_min) + x_min;
			center_y = distribution(engine) / 10000.*(y_max - y_min) + y_min;
#ifdef ALL_FLAT
			yaw = 0;
#else
			if (distribution(engine) / 10000. < 0.5) {
				yaw = 1.57;
			}
			else {
				yaw = 0;
			}
#endif
#ifdef NOT_AXIS_ALIGNED
			yaw = distribution(engine) / 10000. * 3.14;
#endif
#ifdef DIFFERENT_SIZE
			double different_length = (distribution(engine) / 10000. * 2.5 + 1) * OBJ_LENGTH;
			double different_width = OBJ_WIDTH;
			generatePoly(tp, center_x, center_y, yaw, different_length, different_width);
			
#else
#ifdef SPECIAL_STRUCTURE
			generateTetrisBlock(tp, center_x, center_y, yaw, 0);
#else
			generatePoly(tp, center_x, center_y, yaw);
#endif
#endif
			for (auto p = created_polys.begin(); p != created_polys.end(); p++) {
				if (bg::intersects(tp, *p)) {
					isValid = false;
					break;
				}
			}
			

			if (!isValid) {
				continue;
			}

			created_polys.push_back(tp);
			addIntoEnv(tp, global_index);
			global_index++;
			std::vector<Polygon_2> exist_polys;
			for (int j = 0; j < cluster_num_list[i] - 1; j++) {
				std::cout << "obj num " << j << std::endl;
				exist_polys.push_back(tp);
#ifdef SPECIAL_STRUCTURE
				tp = addNewNearTetris(exist_polys, created_polys);
#else
				tp = addNewNearPoly(exist_polys, created_polys);
#endif
				std::cout << "poly size:" << tp.outer().size() << std::endl;
				created_polys.push_back(tp);

				addIntoEnv(tp, global_index);
				global_index++;
			}
		}
	}



	// Then read the bounding rectangle (configuration space)
	int numberOfBoundingVertex;
	Point_2 p;
	p.set<0>(0);
	p.set<1>(0);
	bg::append(m_boundingPoly.outer(), p);
#ifdef BIG_ENV
	p.set<0>(0);
	p.set<1>(2500);
	bg::append(m_boundingPoly.outer(), p);
	p.set<0>(2500);
	p.set<1>(2500);
	bg::append(m_boundingPoly.outer(), p);
	p.set<0>(2500);
	p.set<1>(0);
#endif
#ifdef SMALL_ENV
	p.set<0>(0);
	p.set<1>(1000);
	bg::append(m_boundingPoly.outer(), p);
	p.set<0>(1000);
	p.set<1>(1000);
	bg::append(m_boundingPoly.outer(), p);
	p.set<0>(1000);
	p.set<1>(0);
#endif
	bg::append(m_boundingPoly.outer(), p);
	bg::correct(m_boundingPoly);

	m_envPolyVoronoiList.push_back(m_boundingPoly);
	// Add to scence

	m_pBoundingPolyAGI = new AdvancedGraphicsItem<Polygon_2>(&m_boundingPoly);
	m_pBoundingPolyAGI->m_bFill = false;
	m_pBoundingPolyAGI->m_bShowVertices = false;
	m_pBoundingPolyAGI->m_bShowEdge = true;
	m_pBoundingPolyAGI->m_edgePen = QPen(Qt::black, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
	m_PolyAGIList.push_back(m_pBoundingPolyAGI);
	m_scene.addItem(m_pBoundingPolyAGI);

	// Then compute the outside bounding rectangle
	double x1, y1, x2, y2;
	std::vector<Point_2> const& points = m_boundingPoly.outer();

	x1 = points[0].get<0>(); y1 = points[0].get<1>();
	x2 = points[2].get<0>(); y2 = points[2].get<1>();
	bg::append(m_boundingPoly2.outer(), Point_2(x1 - radius, y1 - radius));
	bg::append(m_boundingPoly2.outer(), Point_2(x1 - radius, y2 + radius));
	bg::append(m_boundingPoly2.outer(), Point_2(x2 + radius, y2 + radius));
	bg::append(m_boundingPoly2.outer(), Point_2(x2 + radius, y1 - radius));
	//	bg::append(m_boundingPoly2.outer(), Point_2(x1 - radius, y1 - radius));
	bg::correct(m_boundingPoly2);
	// Add to scene
	m_pBoundingPolyAGI2 = new AdvancedGraphicsItem<Polygon_2>(&m_boundingPoly2);
	m_pBoundingPolyAGI2->m_bFill = false;
	m_pBoundingPolyAGI2->m_bShowVertices = false;
	m_pBoundingPolyAGI2->m_bShowEdge = true;
	m_pBoundingPolyAGI2->m_edgePen = QPen(Qt::black, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
	m_PolyAGIList.push_back(m_pBoundingPolyAGI2);
	// m_scene.addItem(m_pBoundingPolyAGI2);
	std::string output_filename = "C:/Users/wei/Desktop/optimal_robot_picking/trunk/optimal_robot_picking/test-envs/testcases/tetris/";

	std::ofstream ofs(output_filename + std::to_string((long long)numberOfPolygons)  + "_"+ std::to_string((long long)current_case) + ".txt", std::ofstream::out);
	ofs << 30 << std::endl;
	ofs << numberOfPolygons << std::endl;
	for (int q = 0; q < created_polys.size(); q++) {
		ofs << created_polys[q].outer().size() - 1 << std::endl;
		for (int v = 0; v < created_polys[q].outer().size() - 1; v++) {
			ofs << created_polys[q].outer()[v].get<0>() << " " << created_polys[q].outer()[v].get<1>() << std::endl;
		}
	}
	ofs << 4 << std::endl;
#ifdef BIG_ENV
	ofs << 0 << " " << 0 << std::endl;
	ofs << 0 << " " << 2500 << std::endl;
	ofs << 2500 << " " << 2500 << std::endl;
	ofs << 2500 << " " << 0 << std::endl;
#endif
#ifdef SMALL_ENV
	ofs << 0 << " " << 0 << std::endl;
	ofs << 0 << " " << 1000 << std::endl;
	ofs << 1000 << " " << 1000 << std::endl;
	ofs << 1000 << " " << 0 << std::endl;
#endif
	ofs.close();

	m_radius = radius;

	drawBasicEnvironment();

	// Do roadmap building setup
	m_roadmap.buildRoadmap(&m_envObsPolyList, &m_envPolyList, &m_boundingPoly, &m_envPolyVoronoiList, radius, m_scene);
	// m_roadmap.addToScene(m_scene);

	m_boundingRect = QRectF(x1 - radius * 4, y1 - radius * 4, x2 + radius * 4, y2 + radius * 4);
	current_case++;
	std::cout << "current :" << current_case << std::endl;
#ifdef REPEAT
}
#endif
	// Fit to the view
	fitView();

	// Enable the actions for building the graph
	m_state = BUILD_STATE_MAP_OPEN;
	m_pOverlayLatticAction->setEnabled(true);
	m_pLocateBoundaryAction->setEnabled(true);
	//m_pCreateAction->setEnabled(false);
	m_pSolveAction->setEnabled(true);
	m_pPlayAction->setEnabled(false);

}

void MainWindow::generateOptimalRatioTest(){
    int i = 0;
    bool current_valid = true;
    vector<int> svv;
    vector<int> gvv;
    vector<Point_2> svvPt;
    vector<Point_2> gvvPt;
    int count = 0;
    while(i < 100){
        count = 50000;
        current_valid = true;
        restore();
        m_svVec.clear();
        m_gvVec.clear();
        m_pathMap.clear();
        m_pPathLineItems.clear();
        m_sgVec.clear();
        // Get number of robots
        m_numRobots = m_pLineEdit->text().toInt();
        m_numRobots = 1;
        double spacing = m_pMinDistLineEdit->text().toDouble();

        srand (time(NULL));

        // Setup problem 
        vector<pair<double, double> > vvg;

        svv.clear();
        gvv.clear();
        svvPt.clear();
        gvvPt.clear();
        while(true){
            try{
                // Create random start and goal vertices
                Sleep(600);
                m_svVec.clear(); m_gvVec.clear(); svv.clear(); gvv.clear();
                m_roadmap.createRandomStartGoalPairs(m_numRobots, spacing, m_svVec, m_gvVec);
                //m_roadmap.createRandomStartGoalPairsAtRegion(m_numRobots, spacing, m_svVec, m_gvVec, 0, 0.25, 1, 0, 0.75, 1, 1, 0);
                break;
            }
            catch(...){
            }
        }
        cout<<"m_svVec:"<<endl;
        for(int k = 0; k < m_svVec.size();k++){
            cout<<"("<<m_svVec[k].first<<","<<m_svVec[k].second<<")"<<endl;
        }
        cout<<"m_gvVec:"<<endl;
        for(int k = 0; k < m_gvVec.size();k++){
            cout<<"("<<m_gvVec[k].first<<","<<m_gvVec[k].second<<")"<<endl;
        }
        writeRawfiles(i);
        // QFont font;
        // QPainterPath path;
        // font.setPointSizeF(m_radius/1.5);
        // font.setBold(true);
        // for(int r = 0; r < m_numRobots; r ++){
        //     // Robots at current (start) locations
        //     QGraphicsEllipseItem *prs = m_scene.addEllipse(m_svVec[r].first - m_radius, m_svVec[r].second - m_radius, m_radius*2-0.2, m_radius*2-0.2, 
        //         QPen(Qt::black, 0.2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin),
        //         QBrush(Qt::blue));
        //     prs->setZValue(10);
        //     m_pRobotItemVec.push_back(prs);

        //     // Labels for robots at current (start) locations
        //     QGraphicsSimpleTextItem *ti = m_scene.addSimpleText(QString::number(r+1), font);
        //     ti->setPos(m_svVec[r].first - m_radius/2*(3.5/(m_radius+1)) + (r < 9 ? m_radius / 6 *(2.2/(m_radius-0.3)) :(r > 99 ? -m_radius / 6 *(2.2/(m_radius-0.3)) :0 )) , m_svVec[r].second - m_radius/2*(2./(m_radius-0.5)));
        //     ti->setPen(QPen(QColor(Qt::white), 0.15, Qt::SolidLine, Qt::RoundCap,Qt::RoundJoin));
        //     ti->setZValue(15);
        //     m_pTextItemVec.push_back(ti);

        //     // Robots at goal locations
        //     QGraphicsEllipseItem *prg = m_scene.addEllipse(m_gvVec[r].first - m_radius, m_gvVec[r].second - m_radius, m_radius*2-0.2, m_radius*2-0.2, 
        //         QPen(QColor(127, 127, 127, 64), 0.2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin),
        //         QBrush(QColor(255, 127, 127, 64)));
        //     prg->setZValue(5);

        //     // Labels for robots at goal locations
        //     ti = m_scene.addSimpleText(QString::number(r+1), font);
        //     ti->setPos(m_gvVec[r].first - m_radius/2*(3.5/(m_radius+1)) + (r < 9 ? m_radius / 6 *(2.2/(m_radius-0.3)) :(r > 99 ? -m_radius / 6 *(2.2/(m_radius-0.3)) :0 )), m_gvVec[r].second - m_radius/2*(2./(m_radius-0.5)));
        //     ti->setPen(QPen(QColor(Qt::red), 
        //         0.15, Qt::SolidLine, Qt::RoundCap,Qt::RoundJoin));
        //     ti->setZValue(6);
        // }
        locateBoundingLatticeCycle(false);
        m_roadmap.buildVisibilityGraph();
        double maxLength = 0;
        double length = 0;
        for(int r = 0; r < m_numRobots; r ++){
            vector<pair<double, double> > path;
            length = m_roadmap.computeShortestPath(m_svVec[r].first, m_svVec[r].second, m_gvVec[r].first, m_gvVec[r].second, path);
            if(length > maxLength){ maxLength = length; }
        }
        m_shotestPathLength = maxLength;
        //if(m_shotestPathLength > 1250)continue;
        m_state = BUILD_STATE_MAP_OPEN;
        
        m_roadmap.snapToGraph(m_svVec, svv, svvPt);
        m_roadmap.snapToGraph(m_gvVec, gvv, gvvPt);
        m_largestSnapLength = 0;
        double snap = 0;
        for(int r = 0;  r < m_numRobots; r ++){
            double x = m_svVec[r].first;
            double y = m_svVec[r].second;
            Point_2 p0 = Point_2(x, y);
            x = m_gvVec[r].first;
            y = m_gvVec[r].second;
            Point_2 p1 = Point_2(x, y);
            snap += bg::distance(p0, svvPt[r]);
            snap += bg::distance(p1, gvvPt[r]);
            if(snap > m_largestSnapLength){
                m_largestSnapLength = snap;
            }
            snap = 0;
            m_sgVec.push_back(pair<int, int>(svv[r], gvv[r]));
        }
        writeToTestfiles(i, "_noconnection", m_largestSnapLength, m_shotestPathLength);   
        m_state = BUILD_STATE_MAP_OPEN;
        m_sgVec.clear();
       
        i++;
    }
            // Snap to graph
        
     
    
}


void MainWindow::generateTestcases(){
    int i = 0;
    bool current_valid = true;
    vector<int> svv;
    vector<int> gvv;
    vector<Point_2> svvPt;
    vector<Point_2> gvvPt;
    int count = 0;
    while(i < 10){
        count = 50000;
        current_valid = true;
        restore();
        m_svVec.clear();
        m_gvVec.clear();
        m_pathMap.clear();
        m_pPathLineItems.clear();
        m_sgVec.clear();
        // Get number of robots
        m_numRobots = m_pLineEdit->text().toInt();
        double spacing = m_pMinDistLineEdit->text().toDouble();

        srand (time(NULL));

        // Setup problem 
        vector<pair<double, double> > vvg;

		svv.clear();
        gvv.clear();
        svvPt.clear();
        gvvPt.clear();
        while(true){
            try{
                // Create random start and goal vertices
				Sleep(600);
                m_svVec.clear(); m_gvVec.clear(); svv.clear(); gvv.clear();
               // m_roadmap.createRandomStartGoalPairs(m_numRobots, spacing, m_svVec, m_gvVec);
			    m_roadmap.createRandomStartGoalPairsAtRegion(m_numRobots, spacing, m_svVec, m_gvVec, 0, 0.25, 1, 0, 0.75, 1, 1, 0);
                break;
            }
            catch(...){
            }
        }
        cout<<"m_svVec:"<<endl;
        for(int k = 0; k < m_svVec.size();k++){
            cout<<"("<<m_svVec[k].first<<","<<m_svVec[k].second<<")"<<endl;
        }
        cout<<"m_gvVec:"<<endl;
        for(int k = 0; k < m_gvVec.size();k++){
            cout<<"("<<m_gvVec[k].first<<","<<m_gvVec[k].second<<")"<<endl;
        }
        writeRawfiles(i);
        // QFont font;
        // QPainterPath path;
        // font.setPointSizeF(m_radius/1.5);
        // font.setBold(true);
        // for(int r = 0; r < m_numRobots; r ++){
        //     // Robots at current (start) locations
        //     QGraphicsEllipseItem *prs = m_scene.addEllipse(m_svVec[r].first - m_radius, m_svVec[r].second - m_radius, m_radius*2-0.2, m_radius*2-0.2, 
        //         QPen(Qt::black, 0.2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin),
        //         QBrush(Qt::blue));
        //     prs->setZValue(10);
        //     m_pRobotItemVec.push_back(prs);

        //     // Labels for robots at current (start) locations
        //     QGraphicsSimpleTextItem *ti = m_scene.addSimpleText(QString::number(r+1), font);
        //     ti->setPos(m_svVec[r].first - m_radius/2*(3.5/(m_radius+1)) + (r < 9 ? m_radius / 6 *(2.2/(m_radius-0.3)) :(r > 99 ? -m_radius / 6 *(2.2/(m_radius-0.3)) :0 )) , m_svVec[r].second - m_radius/2*(2./(m_radius-0.5)));
        //     ti->setPen(QPen(QColor(Qt::white), 0.15, Qt::SolidLine, Qt::RoundCap,Qt::RoundJoin));
        //     ti->setZValue(15);
        //     m_pTextItemVec.push_back(ti);

        //     // Robots at goal locations
        //     QGraphicsEllipseItem *prg = m_scene.addEllipse(m_gvVec[r].first - m_radius, m_gvVec[r].second - m_radius, m_radius*2-0.2, m_radius*2-0.2, 
        //         QPen(QColor(127, 127, 127, 64), 0.2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin),
        //         QBrush(QColor(255, 127, 127, 64)));
        //     prg->setZValue(5);

        //     // Labels for robots at goal locations
        //     ti = m_scene.addSimpleText(QString::number(r+1), font);
        //     ti->setPos(m_gvVec[r].first - m_radius/2*(3.5/(m_radius+1)) + (r < 9 ? m_radius / 6 *(2.2/(m_radius-0.3)) :(r > 99 ? -m_radius / 6 *(2.2/(m_radius-0.3)) :0 )), m_gvVec[r].second - m_radius/2*(2./(m_radius-0.5)));
        //     ti->setPen(QPen(QColor(Qt::red), 
        //         0.15, Qt::SolidLine, Qt::RoundCap,Qt::RoundJoin));
        //     ti->setZValue(6);
        // }
        locateBoundingLatticeCycle(false);
        m_roadmap.buildVisibilityGraph();
        double maxLength = 0;
		double length = 0;
        for(int r = 0; r < m_numRobots; r ++){
            vector<pair<double, double> > path;
            length = m_roadmap.computeShortestPath(m_svVec[r].first, m_svVec[r].second, m_gvVec[r].first, m_gvVec[r].second, path);
            if(length > maxLength){ maxLength = length; }
        }
        m_shotestPathLength = maxLength;
        //if(m_shotestPathLength > 1250)continue;
        m_state = BUILD_STATE_MAP_OPEN;
        
        m_roadmap.snapToGraph(m_svVec, svv, svvPt);
        m_roadmap.snapToGraph(m_gvVec, gvv, gvvPt);
        m_largestSnapLength = 0;
        double snap = 0;
        for(int r = 0;  r < m_numRobots; r ++){
            double x = m_svVec[r].first;
            double y = m_svVec[r].second;
            Point_2 p0 = Point_2(x, y);
            x = m_gvVec[r].first;
            y = m_gvVec[r].second;
            Point_2 p1 = Point_2(x, y);
            snap += bg::distance(p0, svvPt[r]);
            snap += bg::distance(p1, gvvPt[r]);
            if(snap > m_largestSnapLength){
                m_largestSnapLength = snap;
            }
            snap = 0;
            m_sgVec.push_back(pair<int, int>(svv[r], gvv[r]));
        }
        writeToTestfiles(i, "_noconnection", m_largestSnapLength, m_shotestPathLength);   
        m_state = BUILD_STATE_MAP_OPEN;
        m_sgVec.clear();
       
        restore();
        locateBoundingLatticeCycle(true);
        svv.clear();
        svvPt.clear();
        gvv.clear();
        gvvPt.clear();
        m_roadmap.snapToGraph(m_svVec, svv, svvPt);
        m_roadmap.snapToGraph(m_gvVec, gvv, gvvPt);
        m_largestSnapLength = 0;
        snap = 0;
        for(int r = 0;  r < m_numRobots; r ++){
            double x = m_svVec[r].first;
            double y = m_svVec[r].second;
            Point_2 p0 = Point_2(x, y);
            x = m_gvVec[r].first;
            y = m_gvVec[r].second;
            Point_2 p1 = Point_2(x, y);
            snap += bg::distance(p0, svvPt[r]);
            snap += bg::distance(p1, gvvPt[r]);
            if(snap > m_largestSnapLength){
                m_largestSnapLength = snap;
            }
            snap = 0;
            
            m_sgVec.push_back(pair<int, int>(svv[r], gvv[r]));
        }
        if(!writeToTestfiles(i, "_connection", m_largestSnapLength, m_shotestPathLength)){
            continue;
        }               
        i++;
    }
            // Snap to graph
        
     
    
}

bool MainWindow::writeRawfiles(int i){
    string pfString = m_fileFolder.toStdString() + "\\raw_" + m_envFileName.toStdString() + "_"+ std::to_string(static_cast<long long>(m_numRobots)) + "_" + std::to_string(static_cast<long long>(i))  + ".txt";
    
   // cout<<"generating "<<fileNameExtra<<":"<<i<<endl;
    ofstream ps(pfString.c_str(), ios::out);
    for(int j = 0; j < m_svVec.size(); j ++){
        ps << m_svVec[j].first << ":" << m_svVec[j].second << " ";
    }
    ps << endl;
    for(int j = 0; j < m_sgVec.size(); j ++){
        ps << m_sgVec[j].first << ":" << m_sgVec[j].second << " ";
    }
    ps << endl;
    ps.close();
	return true;
}

bool MainWindow::writeToTestfiles(int i, string ifconnection, double snapLength, double visibilityLength){
    return m_roadmap.generateTestfile(m_sgVec, m_pathMap, m_fileFolder.toStdString(), m_envFileName.toStdString() + ifconnection, i, m_numRobots, snapLength, visibilityLength);
}

void MainWindow::wholeTest(){
    double makespanLength = 0;
    double smoothLength = 0;
    double optimalRatio = 0;
    double makespan_connection_average = 0;
    double makespan_noconnection_average = 0;
    double smooth_connection_average = 0;
    double smooth_noconnection_average = 0;
    double snapLength = 0;
    double visibilityLength = 0;
    int num = 0;
    int shortcut_times = 0;
    double max_makespan_ratio = 0;
    clock_t tStart = clock();
    double connected_time = 0;
    double unconnected_time = 0;
    int smooth_nums = 0;
    int robot_num = 0;
    ofstream ps;
    for(robot_num = 5; robot_num < 35; robot_num += 5){ 
		num = 0;
		unconnected_time = 0;
		connected_time = 0;
		max_makespan_ratio = 0;
		shortcut_times = 0;
		makespanLength = 0;
		smoothLength = 0;
		optimalRatio = 0;
		makespan_connection_average = 0;
		makespan_noconnection_average = 0;
		smooth_connection_average = 0;
		smooth_noconnection_average = 0;
		snapLength = 0;
		visibilityLength = 0;
	std::string foldername = m_fileFolder.toStdString() + "\\testresult";
    std::string filename = m_fileFolder.toStdString()+"\\testresult"+"\\"+m_envFileName.toStdString()+"_";
    string pfString = filename + std::to_string(static_cast<long long>(robot_num))+".txt";
	cout<<"output file:"<<pfString<<endl;
    ps.open(pfString.c_str(), ios::out);
	
    for(int i = 0;i < 10;i ++){
        //restore();
        //locateBoundingLatticeCycle(true);
        ps<<"solving "<<num+1<<" case"<<endl;
        ps<<"connection case:"<<endl;
        tStart = clock();
        solveGeneratedProblemForRobotNum(i, "_connection", snapLength, visibilityLength, shortcut_times, smooth_nums, robot_num);
        connected_time += (double)(clock() - tStart) / CLOCKS_PER_SEC;
       // ps<<"visibility length is "<<m_shotestPathLength<<endl;
        makespanLength = (m_pathMap[0].size()-1)*m_radius/0.43;
		if (smooth_nums >= m_pathMap[0].size()) {
			smooth_nums = m_pathMap[0].size() - 2;
		}
        smoothLength = (m_pathMap[0].size()-1-shortcut_times*2 - smooth_nums)*m_radius/0.43 + smooth_nums*m_radius/0.43*0.9 + shortcut_times*2*m_radius/0.43*0.866;
        makespanLength += snapLength;
        smoothLength += snapLength;
        ps<<"makespanLength: "<<makespanLength<<endl;
        ps<<"smoothLength: "<<smoothLength<<endl;
        ps<<"visibilityLength:"<<visibilityLength<<endl;
        optimalRatio = makespanLength / visibilityLength;
		//if (optimalRatio > max_makespan_ratio) {
		//	max_makespan_ratio = optimalRatio;
		//}
        if(optimalRatio < 1) ps<<"error happen~~~~~~~~~~~~"<<endl;
        ps<<"makespan optimalRatio is "<<optimalRatio<<endl;
        makespan_connection_average += optimalRatio;
        optimalRatio = smoothLength / visibilityLength;
        //if(optimalRatio < 1) ps<<"error happen~~~~~~~~~~~~"<<endl;
        ps<<"smooth makespan optimalRatio is "<<optimalRatio<<endl;
        smooth_connection_average += optimalRatio;
        //restore();
        //locateBoundingLatticeCycle(false);
        tStart = clock();
        solveGeneratedProblemForRobotNum(i, "_noconnection", snapLength, visibilityLength, shortcut_times, smooth_nums, robot_num);        
        unconnected_time += (double)(clock() - tStart) / CLOCKS_PER_SEC;
        ps<<"noconnection case:"<<endl;
        makespanLength = (m_pathMap[0].size()-1)*m_radius/0.43;
        makespanLength += snapLength;
        smoothLength = (m_pathMap[0].size()-3-shortcut_times*2)*m_radius/0.43*0.91 + shortcut_times*2*m_radius/0.43*0.866;
        ps<<"makespanLength: "<<makespanLength<<endl;
        ps<<"smoothLength: "<<smoothLength<<endl;
        ps<<"visibilityLength:  "<<visibilityLength<<endl;
        optimalRatio = makespanLength / visibilityLength;
        if(optimalRatio < 1) ps<<"error happen~~~~~~~~~~~~"<<endl;
        ps<<"makespan optimalRatio is "<<optimalRatio<<endl;
        makespan_noconnection_average += optimalRatio;
        optimalRatio = smoothLength / visibilityLength;
       // if(optimalRatio < 1) ps<<"error happen~~~~~~~~~~~~"<<endl;
        ps<<"smooth makespan optimalRatio is "<<optimalRatio<<endl;
        smooth_noconnection_average += optimalRatio;
        num ++;
        //while(getchar()== ' ');
    }
    ps<<"(connected)Average time taken: "<< connected_time / 10<<endl;
    ps<<"(unconnected)Average time taken: "<< unconnected_time / 10<<endl;
    makespan_connection_average = makespan_connection_average / num;
    makespan_noconnection_average = makespan_noconnection_average / num;
    smooth_connection_average = smooth_connection_average / num;
    smooth_noconnection_average = smooth_noconnection_average / num;   
    ps<<"makespan connection optimal ratio average:"<< makespan_connection_average << endl;
    ps<<"makespan no connection optimal ratio average:"<< makespan_noconnection_average << endl;
    ps<<"smooth connection optimal ratio average:"<< smooth_connection_average << endl;
    ps<<"smooth no connection optimal ratio average:"<< smooth_noconnection_average << endl;
    ps.close();
    }
}

void MainWindow::optimalRatioTest(){
double makespanLength = 0;
    double smoothLength = 0;
    double optimalRatio = 0;
    double makespan_connection_average = 0;
    double makespan_noconnection_average = 0;
    double smooth_connection_average = 0;
    double smooth_noconnection_average = 0;
    double snapLength = 0;
    double visibilityLength = 0;
    int num = 0;
    int shortcut_times = 0;
    double max_makespan_ratio = 0;
    clock_t tStart = clock();
    double connected_time = 0;
    double unconnected_time = 0;
    int smooth_nums = 0;
    int robot_num = 0;
    

    std::string foldername = m_fileFolder.toStdString() + "\\testresult";
    std::string filename = m_fileFolder.toStdString()+"\\testresult"+"\\"+m_envFileName.toStdString()+"_";
    string pfString = filename + std::to_string(static_cast<long long>( m_numRobots))+".txt";
    cout<<"output file:"<<pfString<<endl;
    ofstream ps(pfString.c_str(), ios::out);
    
    for(int i = 0;i < 100;i ++){
        //restore();
        //locateBoundingLatticeCycle(true);
        ps<<"solving "<<num+1<<" case"<<endl;
        
        //restore();
        //locateBoundingLatticeCycle(false);
        tStart = clock();
        solveGeneratedProblem(i, "_noconnection", snapLength, visibilityLength, shortcut_times, smooth_nums);        
        unconnected_time += (double)(clock() - tStart) / CLOCKS_PER_SEC;
        ps<<"noconnection case:"<<endl;
        makespanLength = (m_pathMap[0].size()-1)*m_radius/0.43;
        makespanLength += snapLength;
       // smoothLength = (m_pathMap[0].size()-3-shortcut_times*2)*m_radius/0.43*0.91 + shortcut_times*2*m_radius/0.43*0.866;
        ps<<"makespanLength: "<<makespanLength<<endl;
        //ps<<"smoothLength: "<<smoothLength<<endl;
        ps<<"visibilityLength:  "<<visibilityLength<<endl;
        optimalRatio = makespanLength / visibilityLength;
        if(optimalRatio < 1) ps<<"error happen~~~~~~~~~~~~"<<endl;
        ps<<"makespan optimalRatio is "<<optimalRatio<<endl;
        makespan_noconnection_average += optimalRatio;
       
        num ++;
        //while(getchar()== ' ');
    }
    //ps<<"(connected)Average time taken: "<< connected_time / 10<<endl;
    //ps<<"(unconnected)Average time taken: "<< unconnected_time / 10<<endl;
    
    makespan_noconnection_average = makespan_noconnection_average / num;
    
    ps<<"makespan no connection optimal ratio average:"<< makespan_noconnection_average << endl;
    
}

void MainWindow::singleTest(){
    double makespanLength = 0;
    double smoothLength = 0;
    double optimalRatio = 0;
    double makespan_connection_average = 0;
    double makespan_noconnection_average = 0;
    double smooth_connection_average = 0;
    double smooth_noconnection_average = 0;
    double snapLength = 0;
    double visibilityLength = 0;
    int num = 0;
    int shortcut_times = 0;
    double max_makespan_ratio = 0;
    clock_t tStart = clock();
    double connected_time = 0;
    double unconnected_time = 0;
    int smooth_nums = 0;
    int robot_num = 0;
    

    std::string foldername = m_fileFolder.toStdString() + "\\testresult";
    std::string filename = m_fileFolder.toStdString()+"\\testresult"+"\\"+m_envFileName.toStdString()+"_";
    string pfString = filename + std::to_string(static_cast<long long>( m_numRobots))+".txt";
    cout<<"output file:"<<pfString<<endl;
    ofstream ps(pfString.c_str(), ios::out);
    
    for(int i = 0;i < 10;i ++){
        //restore();
        //locateBoundingLatticeCycle(true);
        ps<<"solving "<<num+1<<" case"<<endl;
        ps<<"connection case:"<<endl;
        tStart = clock();
        solveGeneratedProblem(i, "_connection", snapLength, visibilityLength, shortcut_times, smooth_nums);
        connected_time += (double)(clock() - tStart) / CLOCKS_PER_SEC;
       // ps<<"visibility length is "<<m_shotestPathLength<<endl;
        makespanLength = (m_pathMap[0].size()-1)*m_radius/0.43;
        smoothLength = (m_pathMap[0].size()-3-shortcut_times*2 - smooth_nums)*m_radius/0.43 + smooth_nums*m_radius/0.43*0.9 + shortcut_times*2*m_radius/0.43*0.866;
        makespanLength += snapLength;
        smoothLength += snapLength;
        ps<<"makespanLength: "<<makespanLength<<endl;
        ps<<"smoothLength: "<<smoothLength<<endl;
        ps<<"visibilityLength:"<<visibilityLength<<endl;
        optimalRatio = makespanLength / visibilityLength;
        //if (optimalRatio > max_makespan_ratio) {
        //  max_makespan_ratio = optimalRatio;
        //}
        if(optimalRatio < 1) ps<<"error happen~~~~~~~~~~~~"<<endl;
        ps<<"makespan optimalRatio is "<<optimalRatio<<endl;
        makespan_connection_average += optimalRatio;
        optimalRatio = smoothLength / visibilityLength;
        //if(optimalRatio < 1) ps<<"error happen~~~~~~~~~~~~"<<endl;
        ps<<"smooth makespan optimalRatio is "<<optimalRatio<<endl;
        smooth_connection_average += optimalRatio;
        //restore();
        //locateBoundingLatticeCycle(false);
        tStart = clock();
        solveGeneratedProblem(i, "_noconnection", snapLength, visibilityLength, shortcut_times, smooth_nums);        
        unconnected_time += (double)(clock() - tStart) / CLOCKS_PER_SEC;
        ps<<"noconnection case:"<<endl;
        makespanLength = (m_pathMap[0].size()-1)*m_radius/0.43;
        makespanLength += snapLength;
        smoothLength = (m_pathMap[0].size()-3-shortcut_times*2)*m_radius/0.43*0.91 + shortcut_times*2*m_radius/0.43*0.866;
        ps<<"makespanLength: "<<makespanLength<<endl;
        ps<<"smoothLength: "<<smoothLength<<endl;
        ps<<"visibilityLength:  "<<visibilityLength<<endl;
        optimalRatio = makespanLength / visibilityLength;
        if(optimalRatio < 1) ps<<"error happen~~~~~~~~~~~~"<<endl;
        ps<<"makespan optimalRatio is "<<optimalRatio<<endl;
        makespan_noconnection_average += optimalRatio;
        optimalRatio = smoothLength / visibilityLength;
       // if(optimalRatio < 1) ps<<"error happen~~~~~~~~~~~~"<<endl;
        ps<<"smooth makespan optimalRatio is "<<optimalRatio<<endl;
        smooth_noconnection_average += optimalRatio;
        num ++;
        //while(getchar()== ' ');
    }
    ps<<"(connected)Average time taken: "<< connected_time / 10<<endl;
    ps<<"(unconnected)Average time taken: "<< unconnected_time / 10<<endl;
    makespan_connection_average = makespan_connection_average / num;
    makespan_noconnection_average = makespan_noconnection_average / num;
    smooth_connection_average = smooth_connection_average / num;
    smooth_noconnection_average = smooth_noconnection_average / num;   
    ps<<"makespan connection optimal ratio average:"<< makespan_connection_average << endl;
    ps<<"makespan no connection optimal ratio average:"<< makespan_noconnection_average << endl;
    ps<<"smooth connection optimal ratio average:"<< smooth_connection_average << endl;
    ps<<"smooth no connection optimal ratio average:"<< smooth_noconnection_average << endl;
    //ps.close();
    
}


void MainWindow::solveGeneratedProblemForRobotNum(int i, string ifconnection, double & snapLength, double& visibilityLength, int& shortcut_times, int& smooth_nums, int robot_num){
    m_roadmap.solveGeneratedProblem(m_pathMap, m_fileFolder.toStdString(), m_envFileName.toStdString() + ifconnection, i, robot_num, snapLength, visibilityLength, shortcut_times, smooth_nums);
}

void MainWindow::solveGeneratedProblem(int i, string ifconnection, double & snapLength, double& visibilityLength, int& shortcut_times, int& smooth_nums){
    m_roadmap.solveGeneratedProblem(m_pathMap, m_fileFolder.toStdString(), m_envFileName.toStdString() + ifconnection, i, m_numRobots, snapLength, visibilityLength, shortcut_times, smooth_nums);
}


void MainWindow::runTest() {
	double greedy_total_time = 0;
	double tree_search_total_time = 0;
	double greedy_particle_total_time = 0;
	double greedy_stepahead_total_time = 0;
	double tree_mcts_total_time = 0;
	double multiexit_greedy_total_time = 0;
	double multiexit_tree_total_time = 0;
	double multiexit_separate_greedy_total_time = 0;
	double greedy_total_dist = 0;
	double tree_search_total_dist = 0;
	double greedy_particle_total_dist = 0;
	double greedy_stepahead_total_dist = 0;
	double tree_mcts_total_dist = 0;
	double multiexit_greedy_total_dist = 0;
	double multiexit_tree_total_dist = 0;
	double multiexit_separate_greedy_total_dist = 0;
	double exhaus_dist = 0;
	double greedy_dist = 0;
	double multi_greedy_dist = 0;
	double particle_greedy_dist = 0;
	double mcts_dist = 0;
	double multiexit_greedy = 0;
	double multiexit_tree = 0;
	double multiexit_separate_greedy = 0;
	int number_different_objects = 1;
	std::vector<int> number_objects_list; 
	//number_objects_list.push_back(5);
	//number_objects_list.push_back(10);
	//number_objects_list.push_back(15);
	//number_objects_list.push_back(20);
	number_objects_list.push_back(25);
	//number_objects_list.push_back(100);
	std::ofstream myfile;
	myfile.open("C:/Users/wei/Desktop/optimal_robot_picking/trunk/optimal_robot_picking/test-envs/testcases/tetris/result.txt");
	for (int p = 0; p < number_different_objects; p++) {
		 greedy_total_time = 0;
		 tree_search_total_time = 0;
		 greedy_particle_total_time = 0;
		 greedy_stepahead_total_time = 0;
		 tree_mcts_total_time = 0;
		 multiexit_greedy_total_time = 0;
		 multiexit_tree_total_time = 0;
		 multiexit_separate_greedy_total_time = 0;
		 greedy_total_dist = 0;
		 tree_search_total_dist = 0;
		 greedy_particle_total_dist = 0;
		 greedy_stepahead_total_dist = 0;
		 tree_mcts_total_dist = 0;
		 multiexit_greedy_total_dist = 0;
		 multiexit_tree_total_dist = 0;
		 multiexit_separate_greedy_total_dist = 0;
		 exhaus_dist = 0;
		 greedy_dist = 0;
		 multi_greedy_dist = 0;
		 particle_greedy_dist = 0;
		 mcts_dist = 0;
		 multiexit_greedy = 0;
		 multiexit_tree = 0;
		 multiexit_separate_greedy = 0;

		for (int k = 0; k < 10; k++) {
			std::string output_filename = "C:/Users/wei/Desktop/optimal_robot_picking/trunk/optimal_robot_picking/test-envs/testcases/tetris/";
			
			output_filename += (std::to_string((long long)number_objects_list[p])+"/" + std::to_string((long long)number_objects_list[p]) + "_" +std::to_string((long long)k) + ".txt");
			readProblemFromFile(output_filename);
			clock_t t1, t2;
			float diff = 0;
			float seconds = 0;
			std::cout << "number " << k << std::endl;
			t1 = clock();
			greedy_dist = m_roadmap.declutterUsingGreedy();
			t2 = clock();
			diff = (float)t2 - (float)t1;
			seconds = diff / CLOCKS_PER_SEC;
			std::cout << "greedy time:" << seconds << std::endl;
			greedy_total_time += seconds;
			t1 = clock();
			exhaus_dist = m_roadmap.declutterUsingTruncatedTree(m_numRobots);
			t2 = clock();
			diff = ((float)t2 - (float)t1);
			seconds = diff / CLOCKS_PER_SEC;
			std::cout << "exhaus time:" << seconds << std::endl;
			tree_search_total_time += seconds;
			t1 = clock();
			mcts_dist = m_roadmap.declutterUsingMCTS(m_numRobots);
			t2 = clock();
			diff = (float)t2 - (float)t1;
			seconds = diff / CLOCKS_PER_SEC;
			std::cout << "mcts time:" << seconds << std::endl;
			tree_mcts_total_time += seconds;
			t1 = clock();
			//multi_greedy_dist = m_roadmap.declutterUsingMultipleGreedy(m_numRobots);
			t2 = clock();
			diff = (float)t2 - (float)t1;
			seconds = diff / CLOCKS_PER_SEC;
			std::cout << "multiple greedy time:" << seconds << std::endl;
			greedy_stepahead_total_time += seconds;
			t1 = clock();
			particle_greedy_dist = m_roadmap.declutterUsingParticleGreedy(m_numRobots);
			t2 = clock();
			diff = (float)t2 - (float)t1;
			seconds = diff / CLOCKS_PER_SEC;
			std::cout << "particle greedy time:" << seconds << std::endl;
			greedy_particle_total_time += seconds;
			t1 = clock();
			multiexit_greedy = m_roadmap.declutterMultiExitGreedy();
			t2 = clock();
			diff = (float)t2 - (float)t1;
			seconds = diff / CLOCKS_PER_SEC;
			std::cout << "multi exit greedy time:" << seconds << std::endl;
			multiexit_greedy_total_time += seconds;

			t1 = clock();
			//multiexit_tree = m_roadmap.declutterUsingMultiExitTruncatedTree(m_numRobots);
			t2 = clock();
			diff = (float)t2 - (float)t1;
			seconds = diff / CLOCKS_PER_SEC;
			multiexit_tree_total_time += seconds;

			t1 = clock();
			multiexit_separate_greedy = m_roadmap.declutterMultiExitSeparateGreedy();
			t2 = clock();
			diff = (float)t2 - (float)t1;
			seconds = diff / CLOCKS_PER_SEC;
			multiexit_separate_greedy_total_time += seconds;
			//std::cout << "multi exit tree time:" << seconds << std::endl;
			//exhaus_dist = m_roadmap.declutterUsingSequence(exhaus_seq);
			//greedy_dist = m_roadmap.declutterUsingSequence(greedy_seq);
			//multi_greedy_dist = m_roadmap.declutterUsingSequence(multi_greedy_seq);
			//particle_greedy_dist = m_roadmap.declutterUsingSequence(particle_greedy_seq);
			//mcts_dist = m_roadmap.declutterUsingSequence(mcts_seq);
			std::cout << "exhaus_dist:" << exhaus_dist << "   , greedy_dist:" << greedy_dist << " ,multi greedy dist:" << multi_greedy_dist << "    ,particle greedy dist:" << particle_greedy_dist << std::endl;
			std::cout << "multiexit_greedy:" << multiexit_greedy << " , multiexit_tree:" << multiexit_tree << std::endl;
			greedy_total_dist += greedy_dist;
			tree_search_total_dist += exhaus_dist;
			greedy_particle_total_dist += particle_greedy_dist;
			greedy_stepahead_total_dist += multi_greedy_dist;
			tree_mcts_total_dist += mcts_dist;
			multiexit_tree_total_dist += multiexit_tree;
			multiexit_greedy_total_dist += multiexit_greedy;
			multiexit_separate_greedy_total_dist += multiexit_separate_greedy;
			std::cout << "average up till now TIME: exhaus_time:" << tree_search_total_time / (k + 1) << "   , greedy_time:" << greedy_total_time / (k + 1) << " ,multi greedy time:" << greedy_stepahead_total_time / (k + 1) << "    ,particle greedy time:" << greedy_particle_total_time / (k + 1) << "  ,mcts time:" << tree_mcts_total_time / (k + 1) << std::endl;
			std::cout << "average up till now Time: multiexit_greedy:" << multiexit_greedy_total_time / (k + 1) << ",  multiexit_tree:" << multiexit_tree_total_time / (k + 1) << ",  multiexit_separate_greedy:" << multiexit_separate_greedy_total_time / (k + 1) << std::endl;
			std::cout << "average up till now DIST: exhaus_dist:" << 2 * tree_search_total_dist / (k + 1) << "   , greedy_dist:" << 2 * greedy_total_dist / (k + 1) << " ,multi greedy dist:" << 2 * greedy_stepahead_total_dist / (k + 1) << "    ,particle greedy dist:" << 2 * greedy_particle_total_dist / (k + 1) << "  ,mcts dist:" << 2 * tree_mcts_total_dist / (k + 1) << std::endl;
			std::cout << "average up till now Dist: multiexit_greedy:" << multiexit_greedy_total_dist / (k + 1) << ",  multiexit_tree:" << multiexit_tree_total_dist / (k + 1) << ",  multiexit_separate_greedy:" << multiexit_separate_greedy_total_dist / (k + 1) << std::endl;
			
		}
		myfile << "average up till now TIME: exhaus_time:" << tree_search_total_time / (9 + 1) << "   , greedy_time:" << greedy_total_time / (9 + 1) << " ,multi greedy time:" << greedy_stepahead_total_time / (9 + 1) << "    ,particle greedy time:" << greedy_particle_total_time / (9 + 1) << "  ,mcts time:" << tree_mcts_total_time / (9 + 1) << "\n";
		myfile << "average up till now Time: multiexit_greedy:" << multiexit_greedy_total_time / (9 + 1) << ",  multiexit_tree:" << multiexit_tree_total_time / (9 + 1) << ",  multiexit_separate_greedy:" << multiexit_separate_greedy_total_time / (9 + 1) << "\n";
		myfile << "average up till now DIST: exhaus_dist:" << 2 * tree_search_total_dist / (9 + 1) << "   , greedy_dist:" << 2 * greedy_total_dist / (9 + 1) << " ,multi greedy dist:" << 2 * greedy_stepahead_total_dist / (9 + 1) << "    ,particle greedy dist:" << 2 * greedy_particle_total_dist / (9 + 1) << "  ,mcts dist:" << 2 * tree_mcts_total_dist / (9 + 1) << "\n";
		myfile << "average up till now Dist: multiexit_greedy:" << multiexit_greedy_total_dist / (9 + 1) << ",  multiexit_tree:" << multiexit_tree_total_dist / (9 + 1) << ",  multiexit_separate_greedy:" << multiexit_separate_greedy_total_dist / (9 + 1) << "\n";
	}
		std::cout << "AVERAGE TIME: exhaus_time:" << tree_search_total_time / 10 << "   , greedy_dist:" << greedy_total_time / 10 << " ,multi greedy dist:" << greedy_stepahead_total_time / 10 << "    ,particle greedy dist:" << greedy_particle_total_time / 10 << "  ,mcts dist:" << tree_mcts_total_time / 10 << std::endl;

		std::cout << "AVERAGE DIST: exhaus_dist:" << tree_search_total_dist/10 << "   , greedy_dist:" << greedy_total_dist /10<< " ,multi greedy dist:" << greedy_stepahead_total_dist/10 << "    ,particle greedy dist:" << greedy_particle_total_dist/10<<"  ,mcts dist:"<< tree_mcts_total_dist /10<< std::endl;
}

void MainWindow::solveProblem() {
	// Setup animation. Note that the solution will be computed the first time animatePath()
	// is called
	runTest();
	/* 
	int numberOfPolygons = m_pLineEdit->text().toInt();
	m_pCreateAction->setEnabled(false);
	m_pSolveAction->setEnabled(false);


	clock_t t1, t2;
	float diff = 0;
	float seconds = 0;
	
	t1 = clock();
	std::vector<int> greedy_seq = m_roadmap.declutterUsingGreedy();
	t2 = clock();
	diff = (float)t2 - (float)t1;
	seconds = diff / CLOCKS_PER_SEC;
	std::cout << "greedy time:" << seconds << std::endl;
	t1 = clock();
	std::vector<int> exhaus_seq = m_roadmap.declutterUsingTruncatedTree(numberOfPolygons);
	t2 = clock();
	diff = ((float)t2 - (float)t1);
	seconds = diff / CLOCKS_PER_SEC;
	std::cout << "exhaus time:" << seconds << std::endl;
	t1 = clock();
	std::vector<int> mcts_seq = m_roadmap.declutterUsingMCTS(numberOfPolygons);
	t2 = clock();
	diff = (float)t2 - (float)t1;
	seconds = diff / CLOCKS_PER_SEC;
	std::cout << "mcts time:" << seconds << std::endl;
	
	t1 = clock();
	std::vector<int> multi_greedy_seq = m_roadmap.declutterUsingMultipleGreedy(numberOfPolygons);
	t2 = clock();
	diff = (float)t2 - (float)t1;
	seconds = diff / CLOCKS_PER_SEC;
	std::cout << "multiple greedy time:" << seconds << std::endl;
	t1 = clock();
	std::vector<int> particle_greedy_seq = m_roadmap.declutterUsingParticleGreedy(numberOfPolygons);
	t2 = clock();
	diff = (float)t2 - (float)t1;
	seconds = diff / CLOCKS_PER_SEC;
	std::cout << "particle greedy time:" << seconds << std::endl;
	t1 = clock();
	std::vector<int> multiexit_greedy_seq = m_roadmap.declutterMultiExitGreedy();
	t2 = clock();
	diff = (float)t2 - (float)t1;
	seconds = diff / CLOCKS_PER_SEC;
	std::cout << "multi exit greedy time:" << seconds << std::endl;
	double exhaus_dist = m_roadmap.declutterUsingSequence(exhaus_seq);
	double greedy_dist = m_roadmap.declutterUsingSequence(greedy_seq);
	double multi_greedy_dist = m_roadmap.declutterUsingSequence(multi_greedy_seq);
	double particle_greedy_dist = m_roadmap.declutterUsingSequence(particle_greedy_seq);
	std::cout << "exhaus_dist:" << exhaus_dist << "   , greedy_dist:" << greedy_dist << " ,multi greedy dist:"<<multi_greedy_dist<<"    ,particle greedy dist:"<<particle_greedy_dist<<std::endl;
	m_pPlayAction->setEnabled(true);
	m_pCreateAction->setEnabled(true);
	*/
}

void MainWindow::animate(){
	// Remove all paths line items from scence
	// for(vector<QGraphicsLineItem*>::iterator lii = m_pPathLineItems.begin(); 
	// 	lii != m_pPathLineItems.end(); lii ++)
	// {
	// 	m_scene.removeItem(*lii);
	// }
	// m_pPathLineItems.clear();

	// Disable play action
	m_pPlayAction->setEnabled(false);
	m_pApp->processEvents();

	// Sleep for half a second
	boost::this_thread::sleep(boost::posix_time::milliseconds(500));

	// Put all start to the initial location
	m_aniState = ANI_STATE_FROM_START;
	m_pathStep = 0;
	m_aniStatePercent = 0.0;
	for(int r = 0; r < m_numRobots; r++){
		m_currentStartMap[r] = m_svVec[r];

		m_currentGoalMap[r] = m_pathMap[r][0];
		double x = m_currentStartMap[r].first*(1-m_aniStatePercent) + m_currentGoalMap[r].first*m_aniStatePercent;
		double y = m_currentStartMap[r].second*(1-m_aniStatePercent) + m_currentGoalMap[r].second*m_aniStatePercent;
		m_pRobotItemVec[r]->setPos(x - m_svVec[r].first, y - m_svVec[r].second);
		m_pTextItemVec[r]->setPos(x - m_radius/2*(3.5/(m_radius+1)) + (r < 9 ? m_radius / 6*(2.2/(m_radius-0.3)):0), y - m_radius/2*(2./(m_radius-0.5)));
	}
	m_pApp->processEvents();

	// Sleep for half a second
	boost::this_thread::sleep(boost::posix_time::milliseconds(2000));

	// Start animation
	m_pTimer->start(2);
}

void MainWindow::animatePath(){
	// If we are at the beginning of a stage, set the temp starts and goals
   // double edgeLength = m_roadmap.getEdgeLength();
    int mode = 0;
	if(m_aniStatePercent == 0.0){
		for(int r = 0; r < m_numRobots; r ++){
			switch(m_aniState){
			// Snapping from start to the graph
			case ANI_STATE_FROM_START:
				m_currentStartMap[r] = m_svVec[r];
				m_currentGoalMap[r] = m_pathMap[r][0];
				break;

			case ANI_STATE_GRID_PATH:
                if(m_pathStep == 0){ 
                    mode = 0;  // beginning
                    m_currentPathMap[r] = m_roadmap.getVertexListLocationFromID(m_pathStep, m_pathMap[r][m_pathStep], m_pathMap[r][m_pathStep + 1], m_pathMap[r][m_pathStep], m_pathMap[r][m_pathStep+2], mode);
				}else if(m_pathStep == m_pathMap[0].size()-2){
                    mode = 1; //end
                    m_currentPathMap[r] = m_roadmap.getVertexListLocationFromID(m_pathStep, m_pathMap[r][m_pathStep], m_pathMap[r][m_pathStep + 1], m_pathMap[r][m_pathStep-1], m_pathMap[r][m_pathStep+1], mode);
                }else{
                    mode = 2;  // during the grid
                    m_currentPathMap[r] = m_roadmap.getVertexListLocationFromID(m_pathStep, m_pathMap[r][m_pathStep], m_pathMap[r][m_pathStep + 1], m_pathMap[r][m_pathStep-1], m_pathMap[r][m_pathStep+2], mode);
                   // cout<<"robot "<<r+1<<": current path:"<<endl;
                   // for(int k = 0;k < m_currentPathMap[r].size();k++){
                   //     cout<<"x:"<<m_currentPathMap[r][k].first<<" y:"<<m_currentPathMap[r][k].second<<" ";
                   //    }
                    cout<<endl;
                }
                m_currentStartMap[r] = m_pathMap[r][m_pathStep];
				m_currentGoalMap[r] = m_pathMap[r][m_pathStep + 1];
				break;
                

			case ANI_STATE_TO_GOAL:
				m_currentStartMap[r] = m_pathMap[r][m_pathStep];
				m_currentGoalMap[r] = m_gvVec[r];
				break;
			case ANI_STATE_COMPLETE:
			default:
				return;
			}
		}
        
        if(m_aniState == ANI_STATE_GRID_PATH){ 
            m_lengthAdditiveListMap.clear();
            m_lengthListMap.clear();
        for(int r = 0; r < m_numRobots;r ++){
            
            double totalLength = 0;
            std::vector<std::pair<double, double>> temp = m_currentPathMap[r];
            for(int i = 0;i < temp.size()-1; i++){

                double length = sqrt(pow(temp[i].first - temp[i+1].first, 2) + pow(temp[i].second - temp[i+1].second, 2));
                m_lengthListMap[r].push_back(length);
                totalLength += length;
            }
            for(int i = 0; i < temp.size()-1;i++){
                m_lengthListMap[r][i] = m_lengthListMap[r][i] / totalLength;
                m_lengthAdditiveListMap[r].push_back(0.0);
                for(int j = 0;j <= i; j++){
                    m_lengthAdditiveListMap[r][i] += m_lengthListMap[r][j];
                }
            }


	    }
        }//if(m_aniState == ANI_STATE_GRID_PATH){ 
        
    }

	// Animate
    
    if(m_aniState == ANI_STATE_FROM_START || m_aniState == ANI_STATE_TO_GOAL){
        m_aniStatePercent += 0.0400001;
        m_aniStatePercent += 0.0400001;
        for(int r = 0; r < m_numRobots; r++){
            double x = m_currentStartMap[r].first*(1-m_aniStatePercent) + m_currentGoalMap[r].first*m_aniStatePercent;
            double y = m_currentStartMap[r].second*(1-m_aniStatePercent) + m_currentGoalMap[r].second*m_aniStatePercent;
            m_pRobotItemVec[r]->setPos(x - m_svVec[r].first, y - m_svVec[r].second);
            m_pTextItemVec[r]->setPos(x - m_radius/2*(3.5/(m_radius+1)) + (r < 9 ? m_radius / 6*(2.2/(m_radius-0.3)):0), y - m_radius/2*(2./(m_radius-0.5)));
        }
    }else if(m_aniState == ANI_STATE_GRID_PATH){
        m_aniStatePercent += 0.0400001;
      
        int currentIndex = 0;
        for(int r = 0; r < m_numRobots;r ++){
            if(m_currentStartMap[r] == m_currentGoalMap[r]){
                double x = m_currentStartMap[r].first*(1-m_aniStatePercent) + m_currentGoalMap[r].first*m_aniStatePercent;
                double y = m_currentStartMap[r].second*(1-m_aniStatePercent) + m_currentGoalMap[r].second*m_aniStatePercent;
                m_pRobotItemVec[r]->setPos(x - m_svVec[r].first, y - m_svVec[r].second);
                m_pTextItemVec[r]->setPos(x - m_radius/2*(3.5/(m_radius+1)) + (r < 9 ? m_radius / 6*(2.2/(m_radius-0.3)):0), y - m_radius/2*(2./(m_radius-0.5)));
            }else{ 
                double x, y;
                if(m_aniStatePercent > 1){
                    int size = m_currentPathMap[r].size();
                    x = m_currentPathMap[r][size-1].first;
                    y = m_currentPathMap[r][size-1].second;
                    m_pRobotItemVec[r]->setPos(x - m_svVec[r].first, y - m_svVec[r].second);
                    m_pTextItemVec[r]->setPos(x - m_radius/2*(3.5/(m_radius+1)) + (r < 9 ? m_radius / 6*(2.2/(m_radius-0.3)):0), y - m_radius/2*(2./(m_radius-0.5)));
                }else{
                    std::vector<std::pair<double, double>> temp = m_currentPathMap[r];
                for(int i = 0; i < temp.size()-1;i++){
                    if(m_aniStatePercent < m_lengthAdditiveListMap[r][i]){
                        currentIndex = i;
                   //     std::cout<<"currentindex:"<<currentIndex<<std::endl;
                    //    std::cout<<"m_lengthAdditiveListMap"<<m_lengthAdditiveListMap[r][i]<<std::endl;
                    //    std::cout<<"statepercent:"<<m_aniStatePercent<<std::endl;
                        break;
                    }

                }   
                
                if(currentIndex > 0){
                    double currentStatePercent = m_aniStatePercent - m_lengthAdditiveListMap[r][currentIndex-1];
                    x = m_currentPathMap[r][currentIndex].first*(1-currentStatePercent/m_lengthListMap[r][currentIndex]) + m_currentPathMap[r][currentIndex+1].first*currentStatePercent/m_lengthListMap[r][currentIndex];
                    y = m_currentPathMap[r][currentIndex].second*(1-currentStatePercent/m_lengthListMap[r][currentIndex]) + m_currentPathMap[r][currentIndex+1].second*currentStatePercent/m_lengthListMap[r][currentIndex];
                }else{
                    x = m_currentPathMap[r][currentIndex].first*(1-m_aniStatePercent/m_lengthListMap[r][currentIndex]) + m_currentPathMap[r][currentIndex+1].first*m_aniStatePercent/m_lengthListMap[r][currentIndex];
                    y = m_currentPathMap[r][currentIndex].second*(1-m_aniStatePercent/m_lengthListMap[r][currentIndex]) + m_currentPathMap[r][currentIndex+1].second*m_aniStatePercent/m_lengthListMap[r][currentIndex];
                }
                
                m_pRobotItemVec[r]->setPos(x - m_svVec[r].first, y - m_svVec[r].second);
                m_pTextItemVec[r]->setPos(x - m_radius/2*(3.5/(m_radius+1)) + (r < 9 ? m_radius / 6*(2.2/(m_radius-0.3)):0), y - m_radius/2*(2./(m_radius-0.5)));
                }

                
            }
        }
    }
	
/*
//works with simple grid 
    m_aniStatePercent += 0.100001;
    if(m_aniState == ANI_STATE_FROM_START || m_aniState == ANI_STATE_TO_GOAL) m_aniStatePercent += 0.100001;
    for(int r = 0; r < m_numRobots; r++){
        double x = m_currentStartMap[r].first*(1-m_aniStatePercent) + m_currentGoalMap[r].first*m_aniStatePercent;
        double y = m_currentStartMap[r].second*(1-m_aniStatePercent) + m_currentGoalMap[r].second*m_aniStatePercent;
        m_pRobotItemVec[r]->setPos(x - m_svVec[r].first, y - m_svVec[r].
            second);
        m_pTextItemVec[r]->setPos(x - m_radius/2*(3.5/(m_radius+1)) + (r < 9 ? m_radius / 6*(2.2/(m_radius-0.3)):0), y - m_radius/2*(2./(m_radius-0.5)));
    }
*/
	// Check current stage perecentage
	if(m_aniStatePercent > 1.0){
		m_aniStatePercent = 0.0;
		switch(m_aniState){
		// Snapping from start to the graph
		case ANI_STATE_FROM_START:
			m_aniState = ANI_STATE_GRID_PATH;
			break;
		case ANI_STATE_GRID_PATH:
			m_pathStep ++;
			if(m_pathStep == m_pathMap[0].size() - 1){
				m_aniState = ANI_STATE_TO_GOAL;
			}
			break;
		case ANI_STATE_TO_GOAL:
			m_aniState = ANI_STATE_COMPLETE;
			m_pTimer->stop();
			m_pPlayAction->setEnabled(true);
			break;
		}
	}
}

// Fit the content to the current view port
void MainWindow::fitView(){
    if(m_pBoundingPolyAGI2 != 0){
        m_pView->setSceneRect(m_pBoundingPolyAGI2->boundingRect());
        m_pView->fitInView(m_boundingRect, Qt::KeepAspectRatio);
    }
	//generateTestcases();
	//generateOptimalRatioTest();
	//wholeTest();
	//optimalRatioTest();
    //singleTest();
	//optimalLossTest();
    
}
