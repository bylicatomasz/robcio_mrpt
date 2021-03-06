#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CEllipsoid.h>
#include <mrpt/opengl/CDisk.h>

#include "RobcioLocalizationPF.h"
//*)
// General global variables:



using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::opengl;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace mrpt::utils;
using namespace mrpt::random;
using namespace mrpt::poses;
using namespace mrpt::bayes;
using namespace std;
RobcioLocalizationPF::RobcioLocalizationPF(RobcioData *robcioDataArg)
{
	robcioData=robcioDataArg;
	initFileExport();

};

void RobcioLocalizationPF::do_pf_localization_slam(const std::string &ini_fil)
{
	ASSERT_FILE_EXISTS_(ini_fil)

		CConfigFile	iniFile(ini_fil);

	vector_int			particles_count;	// Number of initial particles (if size>1, run the experiments N times)

	// Load configuration:
	// -----------------------------------------
	string iniSectionName ( "LocalizationExperiment" );


	// Mandatory entries:
	iniFile.read_vector(iniSectionName, "particles_count", vector_int(1,0), particles_count, /*Fail if not found*/true );
	string		OUT_DIR_PREFIX		= iniFile.read_string(iniSectionName,"logOutput_dir","", /*Fail if not found*/true );


	// Non-mandatory entries:
	string		MAP_FILE			= iniFile.read_string(iniSectionName,"map_file","" );
	size_t		rawlog_offset		= iniFile.read_int(iniSectionName,"rawlog_offset",0);
	string		GT_FILE				= iniFile.read_string(iniSectionName,"ground_truth_path_file","");
	int		NUM_REPS			= iniFile.read_int(iniSectionName,"experimentRepetitions",1);
	int		SCENE3D_FREQ		= iniFile.read_int(iniSectionName,"3DSceneFrequency",10);
	bool 		SCENE3D_FOLLOW = iniFile.read_bool(iniSectionName,"3DSceneFollowRobot",true);
	unsigned int	testConvergenceAt   = iniFile.read_int(iniSectionName,"experimentTestConvergenceAtStep",-1);

	bool    	SAVE_STATS_ONLY = iniFile.read_bool(iniSectionName,"SAVE_STATS_ONLY",false);

	bool 		SHOW_PROGRESS_3D_REAL_TIME = iniFile.read_bool(iniSectionName,"SHOW_PROGRESS_3D_REAL_TIME",false);
	int			SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS = iniFile.read_int(iniSectionName,"SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS",1);
	double 		STATS_CONF_INTERVAL = iniFile.read_double(iniSectionName,"STATS_CONF_INTERVAL",0.2);

#if !MRPT_HAS_WXWIDGETS
	SHOW_PROGRESS_3D_REAL_TIME = false;
#endif

	// Default odometry uncertainty parameters in "dummy_odom_params" depending on how fast the robot moves, etc...
	//  Only used for observations-only rawlogs:
	CActionRobotMovement2D::TMotionModelOptions dummy_odom_params;
	dummy_odom_params.modelSelection = CActionRobotMovement2D::mmGaussian;
	dummy_odom_params.gausianModel.minStdXY  = iniFile.read_double("DummyOdometryParams","minStdXY",0.04);
	dummy_odom_params.gausianModel.minStdPHI = DEG2RAD(iniFile.read_double("DummyOdometryParams","minStdPHI", 2.0));


	// PF-algorithm Options:
	// ---------------------------
	CParticleFilter::TParticleFilterOptions		pfOptions;
	pfOptions.loadFromConfigFile( iniFile, "PF_options" );

	// PDF Options:
	// ------------------
	TMonteCarloLocalizationParams	pdfPredictionOptions;
	pdfPredictionOptions.KLD_params.loadFromConfigFile( iniFile, "KLD_options");

	// Metric map options:
	// -----------------------------
	TSetOfMetricMapInitializers				mapList;
	mapList.loadFromConfigFile( iniFile,"MetricMap");



	cout<< "-------------------------------------------------------------\n"
		<< "\t MAP_FILE = \t "      << MAP_FILE << endl
		<< "\t GT_FILE = \t "       << GT_FILE << endl
		<< "\t OUT_DIR_PREFIX = \t "<< OUT_DIR_PREFIX << endl
		<< "\t #particles = \t "    << particles_count << endl
		<< "-------------------------------------------------------------\n";
	pfOptions.dumpToConsole();
	mapList.dumpToConsole();

	// --------------------------------------------------------------------
	//						EXPERIMENT PREPARATION
	// --------------------------------------------------------------------
	CTicTac		tictac,tictacGlobal;
	CSimpleMap	simpleMap;
	CParticleFilter::TParticleFilterStats	PF_stats;

	// Load the set of metric maps to consider in the experiments:
	CMultiMetricMap							metricMap;
	metricMap.setListOfMaps( &mapList );
	mapList.dumpToConsole();

	randomGenerator.randomize();

	// Load the map (if any):
	// -------------------------
	if (MAP_FILE.size())
	{
		ASSERT_( fileExists(MAP_FILE) );

		// Detect file extension:
		// -----------------------------
		string mapExt = lowerCase( extractFileExtension( MAP_FILE, true ) ); // Ignore possible .gz extensions

		if ( !mapExt.compare( "simplemap" ) )
		{
			// It's a ".simplemap":
			// -------------------------
			printf("Loading '.simplemap' file...");
			CFileGZInputStream(MAP_FILE) >> simpleMap;
			printf("Ok\n");

			ASSERT_( simpleMap.size()>0 );

			// Build metric map:
			// ------------------------------
			printf("Building metric map(s) from '.simplemap'...");
			metricMap.loadFromProbabilisticPosesAndObservations(simpleMap);
			printf("Ok\n");
		}
		else if ( !mapExt.compare( "gridmap" ) )
		{
			// It's a ".gridmap":
			// -------------------------
			printf("Loading gridmap from '.gridmap'...");
			ASSERT_( metricMap.m_gridMaps.size()==1 );
			CFileGZInputStream(MAP_FILE) >> (*metricMap.m_gridMaps[0]);
			printf("Ok\n");
		}
		else
		{
			THROW_EXCEPTION_CUSTOM_MSG1("Map file has unknown extension: '%s'",mapExt.c_str());
		}

	}

	// Load the Ground Truth:
	CMatrixDouble	GT(0,0);
	if ( fileExists( GT_FILE ) )
	{
		printf("Loading ground truth file...");
		GT.loadFromTextFile( GT_FILE );
		printf("OK\n");
	}
	else
		printf("Ground truth file: NO\n");


	// Create 3D window if requested:
	CDisplayWindow3DPtr	win3D;
	if (SHOW_PROGRESS_3D_REAL_TIME)
	{
		win3D = CDisplayWindow3D::Create("pf-localization - The MRPT project", 1000, 600);
		win3D->setCameraZoom(20);
		win3D->setCameraAzimuthDeg(-45);


		//	 wxWindow* window  	 = new wxWindow(win3D->getWxObject());
		//	 RobcioWinFrame *scanFrame = new RobcioWinFrame(wxT("Scan range view"),window);

		// wxTheApp->OnRun();

		//	scanFrame->Show(true);

		//win3D->waitForKey();
		//appWin=new RobcioWinApp();
		//(*appWin).runWin(NULL,NULL);
	}

	// Create the 3D scene and get the map only once, later we'll modify only the particles, etc..
	COpenGLScene			scene;
	COccupancyGridMap2D::TEntropyInfo	gridInfo;

	// The gridmap:
	if (metricMap.m_gridMaps.size())
	{
		metricMap.m_gridMaps[0]->computeEntropy( gridInfo );
		printf("The gridmap has %.04fm2 observed area, %u observed cells\n", gridInfo.effectiveMappedArea, (unsigned) gridInfo.effectiveMappedCells );

		{
			CSetOfObjectsPtr plane = CSetOfObjects::Create();
			metricMap.m_gridMaps[0]->getAs3DObject( plane );
			scene.insert( plane );
		}

		if (SHOW_PROGRESS_3D_REAL_TIME)
		{
			COpenGLScenePtr ptrScene = win3D->get3DSceneAndLock();

			CSetOfObjectsPtr plane = CSetOfObjects::Create();
			metricMap.m_gridMaps[0]->getAs3DObject( plane );
			ptrScene->insert( plane );

			ptrScene->enableFollowCamera(true);

			win3D->unlockAccess3DScene();
		}
	}


	for ( vector_int::iterator itNum = particles_count.begin(); itNum!=particles_count.end(); ++itNum )
	{
		int		PARTICLE_COUNT = *itNum;

		printf("Initial PDF: %f particles/m2\n", PARTICLE_COUNT/gridInfo.effectiveMappedArea);


		// Global stats for all the experiment loops:
		int				nConvergenceTests = 0, nConvergenceOK = 0;
		CVectorDouble 	covergenceErrors;
		// --------------------------------------------------------------------
		//					EXPERIMENT REPETITIONS LOOP
		// --------------------------------------------------------------------
		tictacGlobal.Tic();
		for (int repetition = 0; repetition <NUM_REPS; repetition++)
		{
			cout << "\n-------------------------------------------------------------\n"
				<< "      RUNNING FOR "<< PARTICLE_COUNT << " INITIAL PARTICLES  - Repetition " << 1+repetition << " / " << NUM_REPS << "\n"
				<<"-------------------------------------------------------------\n\n";

		
			// The experiment directory is:
			string      sOUT_DIR, sOUT_DIR_PARTS, sOUT_DIR_3D;

			if (!SAVE_STATS_ONLY)
			{
				sOUT_DIR        = format("%s_%03u",OUT_DIR_PREFIX.c_str(),repetition );
				sOUT_DIR_PARTS  = format("%s/particles", sOUT_DIR.c_str());
				sOUT_DIR_3D = format("%s/3D", sOUT_DIR.c_str());

				printf("Creating directory: %s\n",sOUT_DIR.c_str());
				createDirectory( sOUT_DIR );
				ASSERT_(fileExists(sOUT_DIR));
				deleteFiles(format("%s/*.*",sOUT_DIR.c_str()));

				printf("Creating directory: %s\n",sOUT_DIR_PARTS.c_str());
				createDirectory( sOUT_DIR_PARTS );
				ASSERT_(fileExists(sOUT_DIR_PARTS));
				deleteFiles(format("%s/*.*",sOUT_DIR_PARTS.c_str()));

				printf("Creating directory: %s\n",sOUT_DIR_3D.c_str());
				createDirectory( sOUT_DIR_3D );
				ASSERT_(fileExists(sOUT_DIR_3D));
				deleteFiles(format("%s/*.*",sOUT_DIR_3D.c_str()));

				metricMap.m_gridMaps[0]->saveAsBitmapFile(format("%s/gridmap.png",sOUT_DIR.c_str()));
				CFileOutputStream(format("%s/gridmap_limits.txt",sOUT_DIR.c_str())).printf(
					"%f %f %f %f",
					metricMap.m_gridMaps[0]->getXMin(),metricMap.m_gridMaps[0]->getXMax(),
					metricMap.m_gridMaps[0]->getYMin(),metricMap.m_gridMaps[0]->getYMax() );

				// Save the landmarks for plot in matlab:
				if (metricMap.m_landmarksMap)
					metricMap.m_landmarksMap->saveToMATLABScript2D(format("%s/plot_landmarks_map.m",sOUT_DIR.c_str()));
			}

			int						M = PARTICLE_COUNT;
			CMonteCarloLocalization2D  pdf(M);

			// PDF Options:
			pdf.options = pdfPredictionOptions;

			pdf.options.metricMap = &metricMap;

			// Create the PF object:
			CParticleFilter	PF;
			PF.m_options = pfOptions;

			size_t	step = 0;
			size_t rawlogEntry = 0;

			// Initialize the PDF:
			// -----------------------------
			tictac.Tic();
			if ( !iniFile.read_bool(iniSectionName,"init_PDF_mode",false, /*Fail if not found*/true) )
				pdf.resetUniformFreeSpace(
				metricMap.m_gridMaps[0].pointer(),
				0.7f,
				PARTICLE_COUNT ,
				iniFile.read_float(iniSectionName,"init_PDF_min_x",0,true),
				iniFile.read_float(iniSectionName,"init_PDF_max_x",0,true),
				iniFile.read_float(iniSectionName,"init_PDF_min_y",0,true),
				iniFile.read_float(iniSectionName,"init_PDF_max_y",0,true),
				DEG2RAD(iniFile.read_float(iniSectionName,"init_PDF_min_phi_deg",-180)),
				DEG2RAD(iniFile.read_float(iniSectionName,"init_PDF_max_phi_deg",180))
				);
			else
				pdf.resetUniform(
				iniFile.read_float(iniSectionName,"init_PDF_min_x",0,true),
				iniFile.read_float(iniSectionName,"init_PDF_max_x",0,true),
				iniFile.read_float(iniSectionName,"init_PDF_min_y",0,true),
				iniFile.read_float(iniSectionName,"init_PDF_max_y",0,true),
				DEG2RAD(iniFile.read_float(iniSectionName,"init_PDF_min_phi_deg",-180)),
				DEG2RAD(iniFile.read_float(iniSectionName,"init_PDF_max_phi_deg",180)),
				PARTICLE_COUNT
				);


			printf("PDF of %u particles initialized in %.03fms\n", M, 1000*tictac.Tac());

			// -----------------------------
			//		Particle filter
			// -----------------------------
			CPose2D				pdfEstimation, odometryEstimation;
			CMatrixDouble		cov;
			bool				end = false;

			CFileOutputStream   f_cov_est,f_pf_stats,f_odo_est;

			if (!SAVE_STATS_ONLY)
			{
				f_cov_est.open(sOUT_DIR.c_str()+string("/cov_est.txt"));
				f_pf_stats.open(sOUT_DIR.c_str()+string("/PF_stats.txt"));
				f_odo_est.open(sOUT_DIR.c_str()+string("/odo_est.txt"));
			}

			TTimeStamp cur_obs_timestamp;
					//-----------------------------------------
				// Load pose change from the rawlog:
				// ----------------------------------------
			//	CActionCollection out_action;
			//	CSensoryFrame     out_observations;
				CActionCollection action;
				CSensoryFrame     observations;
			while (true)
			{


				while(true){

					
					bool isNewRecord=parseLineDataForSLAM(action,observations);
					
					if(isNewRecord){
							// Load pose change from the rawlog:
					//	action=out_action;
					//	observations=out_observations;
					//	int si=observations.size();
						break;
					}else{
						sleep(100);
					}
				}
		

				CPose2D		expectedPose; // Ground truth

				if (observations.size()>0)
					cur_obs_timestamp = observations.getObservationByIndex(0)->timestamp;

				if (step>=rawlog_offset)
				{
					// Do not execute the PF at "step=0", to let the initial PDF to be
					//   reflected in the logs.
					if (step>rawlog_offset)
					{
						// Show 3D?
						if (SHOW_PROGRESS_3D_REAL_TIME)
						{
							CPose2D       meanPose;
							CMatrixDouble33 cov;
							pdf.getCovarianceAndMean(cov,meanPose);

							if (rawlogEntry>=2)
								getGroundTruth(expectedPose, rawlogEntry-2, GT, cur_obs_timestamp );

							COpenGLScenePtr ptrScene = win3D->get3DSceneAndLock();

							win3D->setCameraPointingToPoint(meanPose.x(),meanPose.y(),0);

							win3D->addTextMessage(
								10,10, mrpt::format("timestamp: %s", mrpt::system::dateTimeLocalToString(cur_obs_timestamp).c_str() ),
								mrpt::utils::TColorf(.8f,.8f,.8f),
								"mono", 15, mrpt::opengl::NICE, 6001 );

							win3D->addTextMessage(
								10,33, mrpt::format("#particles= %7u", static_cast<unsigned int>(pdf.size()) ),
								mrpt::utils::TColorf(.8f,.8f,.8f),
								"mono", 15, mrpt::opengl::NICE, 6002 );

							win3D->addTextMessage(
								10,55, mrpt::format("mean pose (x y phi_deg)= %s", meanPose.asString().c_str() ),
								mrpt::utils::TColorf(.8f,.8f,.8f),
								"mono", 15, mrpt::opengl::NICE, 6003 );

							// The Ground Truth (GT):
							{
								CRenderizablePtr GTpt = ptrScene->getByName("GT");
								if (!GTpt)
								{
									GTpt = CDisk::Create();
									GTpt->setName( "GT" );
									GTpt->setColor(0,0,0, 0.9);

									getAs<CDisk>(GTpt)->setDiskRadius(0.04);
									ptrScene->insert( GTpt );
								}

								GTpt->setPose( expectedPose );
							}


							// The particles:
							{
								CRenderizablePtr parts = ptrScene->getByName("particles");
								if (parts) ptrScene->removeObject(parts);

								CSetOfObjectsPtr p = pdf.getAs3DObject<CSetOfObjectsPtr>();
								p->setName("particles");
								ptrScene->insert(p);
							}

							// The particles' cov:
							{
								CRenderizablePtr	ellip = ptrScene->getByName("parts_cov");
								if (!ellip)
								{
									ellip = CEllipsoid::Create();
									ellip->setName( "parts_cov");
									ellip->setColor(1,0,0, 0.6);

									getAs<CEllipsoid>(ellip)->setLineWidth(2);
									getAs<CEllipsoid>(ellip)->setQuantiles(3);
									getAs<CEllipsoid>(ellip)->set2DsegmentsCount(60);
									ptrScene->insert( ellip );
								}
								ellip->setLocation(meanPose.x(), meanPose.y(), 0.05 );

								getAs<CEllipsoid>(ellip)->setCovMatrix(cov,2);
							}


							// The laser scan:
							{
								CRenderizablePtr scanPts = ptrScene->getByName("scan");
								if (!scanPts)
								{
									scanPts = CPointCloud::Create();
									scanPts->setName( "scan" );
									scanPts->setColor(1,0,0, 0.9);
									getAs<CPointCloud>(scanPts)->enableColorFromZ(false);
									getAs<CPointCloud>(scanPts)->setPointSize(4);
									ptrScene->insert(scanPts);
								}

								CSimplePointsMap	map;
								static CSimplePointsMap	last_map;

								CPose3D				robotPose3D( meanPose );

								map.clear();
								observations.insertObservationsInto( &map );

								getAs<CPointCloud>(scanPts)->loadFromPointsMap( &last_map );
								getAs<CPointCloud>(scanPts)->setPose( robotPose3D );
								last_map = map;
							}

							// The camera:
							ptrScene->enableFollowCamera(true);

							// Views:
							COpenGLViewportPtr view1= ptrScene->getViewport("main");
							{
								CCamera  &cam = view1->getCamera();
								cam.setAzimuthDegrees(-90);
								cam.setElevationDegrees(90);
								cam.setPointingAt( meanPose );
								cam.setZoomDistance(5);
								cam.setOrthogonal();
							}

					

							win3D->unlockAccess3DScene();

							// Move camera:
							//win3D->setCameraPointingToPoint( curRobotPose.x, curRobotPose.y, curRobotPose.z );

							// Update:
							win3D->forceRepaint();

							sleep( SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS );
						} // end show 3D real-time



						// ----------------------------------------
						// RUN ONE STEP OF THE PARTICLE FILTER:
						// ----------------------------------------
						tictac.Tic();
						if (!SAVE_STATS_ONLY)
							printf("Step %u -- Executing ParticleFilter on %u particles....",(unsigned int)step, (unsigned int)pdf.particlesCount());

						PF.executeOn(
							pdf,
							&action,			// Action
							&observations,	// Obs.
							&PF_stats		// Output statistics
							);

						if (!SAVE_STATS_ONLY)
							printf(" Done! in %.03fms, ESS=%f\n", 1000.0f*tictac.Tac(), pdf.ESS());
					}

					// Avrg. error:
					// ----------------------------------------
					CActionRobotMovement2DPtr best_mov_estim = action.getBestMovementEstimation();
					if (best_mov_estim)
						odometryEstimation = odometryEstimation + best_mov_estim->poseChange->getMeanVal();

					pdf.getMean( pdfEstimation );

					getGroundTruth(expectedPose, rawlogEntry, GT, cur_obs_timestamp );
					//cout << format("TIM: %f GT:", mrpt::system::timestampTotime_t(cur_obs_timestamp)) << expectedPose << endl;

#if 1
					{	// Averaged error to GT
						double sumW=0;
						double locErr=0;
						for (size_t k=0;k<pdf.size();k++) sumW+=exp(pdf.getW(k));
						for (size_t k=0;k<pdf.size();k++)
							locErr+= expectedPose.distanceTo( pdf.getParticlePose(k) ) * exp(pdf.getW(k))/ sumW;
						covergenceErrors.push_back( locErr );
					}
#else
					// Error of the mean to GT
					covergenceErrors.push_back( expectedPose.distanceTo( pdfEstimation ) );
#endif

					// Text output:
					// ----------------------------------------
					if (!SAVE_STATS_ONLY)
					{
						cout << "    Odometry est: " << odometryEstimation << "\n";
						cout << "         PDF est: " << pdfEstimation << ", ESS (B.R.)= " << PF_stats.ESS_beforeResample << "\n";
						if (GT.getRowCount()>0)
							cout << "    Ground truth: " << expectedPose << "\n";
					}

					pdf.getCovariance(cov);

					if (!SAVE_STATS_ONLY)
					{
						f_cov_est.printf("%e\n",sqrt(cov.det()) );
						f_pf_stats.printf("%u %e %e\n",
							(unsigned int)pdf.size(),
							PF_stats.ESS_beforeResample,
							PF_stats.weightsVariance_beforeResample );
						f_odo_est.printf("%f %f %f\n",odometryEstimation.x(),odometryEstimation.y(),odometryEstimation.phi());
					}

					CPose2D meanPose;
					CMatrixDouble33 cov;
					pdf.getCovarianceAndMean(cov,meanPose);

					if ( !SAVE_STATS_ONLY && SCENE3D_FREQ>0 && (step % SCENE3D_FREQ)==0)
					{
						// Generate 3D scene:
						// ------------------------------
						MRPT_TODO("Someday I should clean up this mess, since two different 3D scenes are built -> refactor code")

							// The Ground Truth (GT):
						{
							CRenderizablePtr GTpt = scene.getByName("GT");
							if (!GTpt)
							{
								GTpt = CDisk::Create();
								GTpt = CDisk::Create();
								GTpt->setName( "GT" );
								GTpt->setColor(0,0,0, 0.9);

								getAs<CDisk>(GTpt)->setDiskRadius(0.04);
								scene.insert( GTpt );
							}

							GTpt->setPose(expectedPose);
						}

						// The particles:
						{
							CRenderizablePtr parts = scene.getByName("particles");
							if (parts) scene.removeObject(parts);

							CSetOfObjectsPtr p = pdf.getAs3DObject<CSetOfObjectsPtr>();
							p->setName("particles");
							scene.insert(p);
						}

						// The particles' cov:
						{
							CRenderizablePtr	ellip = scene.getByName("parts_cov");
							if (!ellip)
							{
								ellip = CEllipsoid::Create();
								ellip->setName( "parts_cov");
								ellip->setColor(1,0,0, 0.6);

								getAs<CEllipsoid>(ellip)->setLineWidth(4);
								getAs<CEllipsoid>(ellip)->setQuantiles(3);
								getAs<CEllipsoid>(ellip)->set2DsegmentsCount(60);
								scene.insert( ellip );
							}
							ellip->setLocation(meanPose.x(),meanPose.y(),0);

							getAs<CEllipsoid>(ellip)->setCovMatrix(cov,2);
						}


						// The laser scan:
						{
							CRenderizablePtr scanPts = scene.getByName("scan");
							if (!scanPts)
							{
								scanPts = CPointCloud::Create();
								scanPts->setName( "scan" );
								scanPts->setColor(1,0,0, 0.9);
								getAs<CPointCloud>(scanPts)->enableColorFromZ(false);
								getAs<CPointCloud>(scanPts)->setPointSize(4);
								scene.insert(scanPts);
							}

							CSimplePointsMap	map;
							static CSimplePointsMap	last_map;

							CPose3D				robotPose3D( meanPose );

							map.clear();
							observations.insertObservationsInto( &map );

							getAs<CPointCloud>(scanPts)->loadFromPointsMap( &last_map );
							getAs<CPointCloud>(scanPts)->setPose( robotPose3D );
							last_map = map;
						}

						// The camera:
						scene.enableFollowCamera(SCENE3D_FOLLOW);

						// Views:
						COpenGLViewportPtr view1= scene.getViewport("main");
						{
							CCamera  &cam = view1->getCamera();
							cam.setAzimuthDegrees(-90);
							cam.setElevationDegrees(90);
							cam.setPointingAt( meanPose);
							cam.setZoomDistance(5);
							cam.setOrthogonal();
						}

						/*COpenGLViewportPtr view2= scene.createViewport("small_view"); // Create, or get existing one.
						view2->setCloneView("main");
						view2->setCloneCamera(false);
						view2->setBorderSize(3);
						{
						CCamera  &cam = view1->getCamera();
						cam.setAzimuthDegrees(-90);
						cam.setElevationDegrees(90);
						cam.setPointingAt( meanPose );
						cam.setZoomDistance(15);
						cam.setOrthogonal();

						view2->setTransparent(false);
						view2->setViewportPosition(0.59,0.01,0.4,0.3);
						}*/
					}

					if (!SAVE_STATS_ONLY && SCENE3D_FREQ!=-1 && (step % SCENE3D_FREQ)==0)
					{
						// Save 3D scene:
						CFileGZOutputStream(format("%s/progress_%03u.3Dscene",sOUT_DIR_3D.c_str(),(unsigned)step)) << scene;

						// Generate text files for matlab:
						// ------------------------------------
						pdf.saveToTextFile(format("%s/particles_%03u.txt",sOUT_DIR_PARTS.c_str(),(unsigned)step));

						if (IS_CLASS(*observations.begin(),CObservation2DRangeScan))
						{
							CObservation2DRangeScanPtr o = CObservation2DRangeScanPtr( *observations.begin() );
							vectorToTextFile(o->scan , format("%s/observation_scan_%03u.txt",sOUT_DIR_PARTS.c_str(),(unsigned)step) );
						}
					}

				} // end if rawlog_offset

				step++;

				// Test for end condition if we are testing convergence:
				if ( step == testConvergenceAt )
				{
					nConvergenceTests++;

					// Convergence??
					if ( sqrt(cov.det()) < 2 )
					{
						if ( pdfEstimation.distanceTo(expectedPose) < 1.00f )
							nConvergenceOK++;
					}
					end = true;
				}

				action.clear();
				observations.clear();
				
			
			}; // while rawlogEntries

		} // for repetitions

		double repetitionTime = tictacGlobal.Tac();

		// Avr. error:
		double covergenceErrorMean, covergenceErrorsMin,covergenceErrorsMax;
		math::confidenceIntervals(covergenceErrors, covergenceErrorMean, covergenceErrorsMin,covergenceErrorsMax, STATS_CONF_INTERVAL);

		// Save overall results:
		{

			CFileOutputStream f(format("%s_SUMMARY.txt",OUT_DIR_PREFIX.c_str()), true /* append */);

			f.printf("%% Ratio_covergence_success  #particles  average_time_per_execution  convergence_mean_error convergence_error_conf_int_inf convergence_error_conf_int_sup \n");
			if (!nConvergenceTests) nConvergenceTests=1;
			f.printf("%f %u %f %f %f %f\n",
				((double)nConvergenceOK)/nConvergenceTests,
				PARTICLE_COUNT,
				repetitionTime /NUM_REPS,
				covergenceErrorMean,
				covergenceErrorsMin,covergenceErrorsMax );
		}

		printf("\n TOTAL EXECUTION TIME = %.06f sec\n", repetitionTime );

	} // end of loop for different # of particles

	if (win3D)
		mrpt::system::pause();
}

bool RobcioLocalizationPF::parseLineDataForSLAM(CActionCollection &out_action,CSensoryFrame     &out_observations){
			string dataToAdd=RobcioLocalizationPF::robcioData->getData();
			if(!dataToAdd.empty()){
				
					rowNumber++;
					//printf("\n readDataScanFromCSV \n");
					CPose2D odometryIncrements;
					CPose2D odometryIncrementsTest;
					vector<double> changeStatusArrayData=parseLineLog(dataToAdd);
					string actionStep=parseLineGetStat(dataToAdd); 

					double newCompasRadius=changeStatusArrayData[2];
					double overAll=360;
					double radiusFromCompas=overAll*(newCompasRadius);

					if(startTimestamp==0){
						startTimestamp=mrpt::system::time_tToTimestamp(changeStatusArrayData[0]);

					}
					mrpt::system::TTimeStamp timestamp=startTimestamp+mrpt::system::secondsToTimestamp(rowNumber);
					if(actionStep=="Stop" && actionStepLast=="Stop"){		
						newDystanceSum=0.0;
						actionStepLast=actionStep;
						return false;
					}else if(actionStep=="StartRecording" && actionStepLast=="StartRecording"){					
						actionStepLast=actionStep;
						return false;
					}else if(actionStep=="Forward"  && actionStepLast=="Forward" && newDystanceSum<0.15){
						double newDystance=changeStatusArrayData[1];
						newDystanceSum+=newDystance;
						actionStepLast=actionStep;
						return false;
					}else if((actionStep=="Left" || actionStep=="Right") && (actionStepLast=="Left" || actionStepLast=="Right")  && radiusFromCompasSum<10.0){
						//&& radiusFromCompasSum<10.0
						newDystanceSum=0.0;
						radiusFromCompasSum+=abs(radiusFromCompas-radiusFromCompasLast);
						radiusFromCompasLast=radiusFromCompas;
						actionStepLast=actionStep;
						return false;
					}
					
					
					double newDystance=changeStatusArrayData[1];
					newDystanceSum+=newDystance;
					

					double phiCompasRadius=DEG2RAD(radiusFromCompas);
					//	printf("phiCompasRadius: %f , radiusFromCompas: %f  \n",phiCompasRadius,radiusFromCompas);

					vector<double> arrayXY=getCordinate((newDystanceSum), DEG2RAD(overAll*(newCompasRadius)));
					compasRadius=newCompasRadius;
					newDystanceSum=0.0;
					radiusFromCompasSum=0.0;
					radiusFromCompasLast=radiusFromCompas;
					//isStart=false;
					//actionStepLast="Left";
					//actionStep="Left";
					CObservationOdometry cob;
					cob.sensorLabel="RobcioSensor";
					if(isStart){


						odometryIncrements=CPose2D(0,0,phiCompasRadius);		

						isStart=false;

					}else if(actionStep=="Forward" ){
						//}else if(actionStep=="Forward" && (actionStepLast=="Forward" || actionStepLast=="Back" || actionStepLast=="Stop")){
						lastX+=arrayXY[0];
						lastY+=arrayXY[1];
						odometryIncrementsTest=CPose2D(arrayXY[0],arrayXY[1],phiCompasRadius);					

						odometryIncrements=CPose2D(lastX,lastY,phiCompasRadius);					
						//}else if(actionStep=="Back" && (actionStepLast=="Back" || actionStepLast=="Forward" || actionStepLast=="Stop")){
					}else if(actionStep=="Back" ){
						lastX+=arrayXY[0];
						lastY+=arrayXY[1];
						odometryIncrementsTest=CPose2D(arrayXY[0],arrayXY[1],phiCompasRadius);	
						odometryIncrements=CPose2D(lastX,lastY,phiCompasRadius);	
					}else if(actionStep=="Left" && actionStepLast=="Left"){
						odometryIncrementsTest=CPose2D(0,0,phiCompasRadius);	
						odometryIncrements=CPose2D(lastX,lastY,phiCompasRadius);
					}else if(actionStep=="Right" && actionStepLast=="Right"){
						odometryIncrementsTest=CPose2D(0,0,phiCompasRadius);	
						odometryIncrements=CPose2D(lastX,lastY,phiCompasRadius);
					}else{
						odometryIncrementsTest=CPose2D(0,0,phiCompasRadius);	
						odometryIncrements=CPose2D(lastX,lastY,phiCompasRadius);


					}
					actionStepLast=actionStep;



					cob.odometry=odometryIncrements;		

					//printf("X: %f , Y: %f cord: %s \n",arrayXY[0],arrayXY[1],cob.odometry.asString().c_str());
					cob.timestamp=timestamp;
					CObservationOdometryPtr odom = CObservationOdometryPtr(new CObservationOdometry(cob));


					//printf(" cord: %s %s %i \n",odometryIncrements.asString().c_str(),actionStep.c_str(),radiusFromCompas);

					/*
					*The scan
					*/

					CObservationBearingRange	m_lastObservation;
					CObservationBearingRange obsRange;
					obsRange.maxSensorDistance=2.5;
					obsRange.timestamp = mrpt::system::time_tToTimestamp(changeStatusArrayData[0]);

					//obs.insertObservationInto(

					CObservation2DRangeScanPtr the_scan = CObservation2DRangeScan::Create();
					the_scan->rightToLeft=false;
					the_scan->aperture = DEG2RAD(120);
					the_scan->timestamp = timestamp;
					//the_scan->sensorPose=cob.odometry; // this is not workin I get wrong map
					the_scan->sensorLabel="RobcioSensor";
					the_scan->maxRange=2.50f;

					vector<CObservationBearingRange::TMeasurement> dataM;
					for(int i=7;i<changeStatusArrayData.size();i++){

						CObservationBearingRange::TMeasurement messs;
						messs.landmarkID=lendmarkId;
						messs.pitch=0;
						messs.range=changeStatusArrayData[i];
						messs.yaw=DEG2RAD(90.0F-(30.0f +((i-7.0f))/2.0f));
						//messs.yaw=DEG2RAD(((60.0F+changeStatusArrayData.size()-(i-7.0f))/2.0f));
						m_lastObservation.sensedData.push_back(messs);

						lendmarkId++;
						//	m_lastObservation.sensedData[i].range=changeStatusArrayData[i];
						//    m_lastObservation.sensedData[i].yaw=DEG2RAD((i-7)/2);

						if(changeStatusArrayData[i]>the_scan->maxRange){

							the_scan->scan.push_back(changeStatusArrayData[i]);
							the_scan->validRange.push_back(0);
						}else{
							the_scan->scan.push_back(changeStatusArrayData[i]);
							vector<double> arrayMapXY=getCordinate(messs.range,DEG2RAD(overAll*(newCompasRadius))+ messs.yaw);
							robcioData->putDataMapXY(odoLast.x()+arrayMapXY[0],odoLast.y()+arrayMapXY[1]);						
							the_scan->validRange.push_back(1);
						}

					}



					//pathX.push_back(odoLast.x());
					//pathY.push_back(odoLast.y());
					CActionCollection    acts;
					//CActionRobotMovement2D move;


					CActionRobotMovement2D	act;
					CActionRobotMovement2D::TMotionModelOptions	opts;
					opts.modelSelection = CActionRobotMovement2D::mmGaussian;

					CPose2D  Aodom =cob.odometry- odoLast;

					act.computeFromOdometry(Aodom, opts);
					act.timestamp = timestamp;

					acts.insert(act);

					odoLast=cob.odometry;
					CSensoryFrame frame;


					CObservationBearingRange *rang= new CObservationBearingRange(m_lastObservation);
					rang->timestamp= timestamp;
					frame.insert(the_scan);
					frame.insert( CObservationBearingRangePtr(rang ));


					robcioData->putDataPathXY(odoLast.x(),odoLast.y());	
					out_file << frame;
					out_file <<acts;
					

					out_action=acts;
					out_observations=frame;


					int ile=out_observations.size();
					//out_file << odom;
				return true;
			}
			return false;
}



void  RobcioLocalizationPF:: getGroundTruth( CPose2D &expectedPose, size_t rawlogEntry, const CMatrixDouble &GT, const TTimeStamp &cur_time)
{
	if (GT.getColCount()==4)
	{
		static bool first_step = true;
		static bool GT_index_is_time;

		// First column can be: timestamps, or rawlogentries:
		//  Auto-figure it out:
		if (GT.getRowCount()>2)
		{
			GT_index_is_time = floor(GT(0,0))!=GT(0,0) && floor(GT(1,0))!=GT(1,0);
		}
		else
		{
			GT_index_is_time = false;
		}

		if (GT_index_is_time)
		{
			// Look for the timestamp:
			static std::map<double,CPose2D>	GT_path;
			std::map<double,CPose2D>::iterator it;
			if (first_step)
			{
				for (size_t i=0;i<GT.getRowCount();i++)
					GT_path[ mrpt::utils::round_10power(GT(i,0),-4) ] = CPose2D(GT(i,1),GT(i,2),GT(i,3));
			}

			double TT =mrpt::system::timestampTotime_t(cur_time);
			double T = mrpt::utils::round_10power( TT, -4);

			it = GT_path.find(T);
			if (it!=GT_path.end())
			{
				expectedPose = it->second;
			}
			else cout << format("GT time not found: %f\n", T);
		}
		else
		{
			// Look for the rawlogEntry:
			size_t  k, N = GT.getRowCount();
			for (k=0;k<N;k++)
			{
				if (GT(k,0)==rawlogEntry )
					break;
			}

			if (k<N)
			{
				expectedPose.x(GT(k,1));
				expectedPose.y(GT(k,2));
				expectedPose.phi(GT(k,3));
			}
		}
		first_step=false;
	}
	else
		if (GT.getColCount()==3)
		{
			if ( rawlogEntry<GT.getRowCount() )
			{
				expectedPose.x(GT(rawlogEntry,0) );
				expectedPose.y( GT(rawlogEntry,1) );
				expectedPose.phi( GT(rawlogEntry,2) );
			}
		}
		else if (GT.getColCount()>0) THROW_EXCEPTION("Unexpected number of columns in ground truth file");
}


vector<double>  RobcioLocalizationPF:: parseLineLog(string line){
	vector<double> returnArr;	
	vector<string> arrSplit = split(line, ';');
	for(int i=0;i<arrSplit.size();i++){			
		if(i==4){
			continue;
		}
		replace(arrSplit[i].begin(),arrSplit[i].end(),',','.');				
		double num=atof(arrSplit[i].c_str());
		returnArr.push_back(num);


	}
	return returnArr;
};
string  RobcioLocalizationPF:: parseLineGetStat(string line){
	vector<double> returnArr;	
	vector<string> arrSplit = split(line, ';');
	return arrSplit[4];
};
vector<std::string>  RobcioLocalizationPF:: split(const std::string &s, char delim, std::vector<std::string> &elems) {
	std::stringstream ss(s);
	std::string item;
	while (std::getline(ss, item, delim)) {
		elems.push_back(item);
	}
	return elems;
}
vector<std::string>  RobcioLocalizationPF::split(const std::string &s, char delim) {
	std::vector<std::string> elems;
	split(s, delim, elems);
	return elems;
}
vector<double> RobcioLocalizationPF::getCordinate(double dystance, double radius){
	double x = dystance * cos(radius);
	double y = dystance * sin(radius);


	vector<double>  arrayDouble;

	arrayDouble.push_back(x);
	arrayDouble.push_back(y);

	return arrayDouble;

};

// ------------------------------------------------------
//				Create RawLog
// ------------------------------------------------------
void RobcioLocalizationPF:: printProgress(int *count){
	(*count)++;
	if((*count)%100==0){
		cout << "\n Count: "<< (*count);
		//	printf("\n Count is:  "+std::to_string((*count)));
	}
}
void RobcioLocalizationPF:: importFromCSVFile(){

	printf("\n Start ImportFromCSVFile \n");
	initFileExport();
	ifstream input( file_input_csv );	
	int count=0;
	for( std::string line; getline( input, line ); )
	{
		printProgress(&count);
	//	writeToCSV(line);
		robcioData->putData(line);
		sleep(200);
		//readDataScanFromString(line);

	}
	//closeFileExport();
};
void RobcioLocalizationPF:: initFileExport(){
	
	
	file_input_csv="C:/Robotics/mrpt-1.2.1/apps/RobcioApp/config_files/PartSmallMoved.csv";
	newDystanceSum=0;
	lendmarkId=0;
	lastX=0;
	lastY=0;
	isStart=true;
	compasRadius=0;
	dystance=0;
	startTimestamp=0;
	rowNumber=0;
	odoLast=CPose2D(0,0,0);
	file_rawlog_output="C:\\Temp\\robci_app_dataset_"+currentDateTime()+".rawlog";
	file_csv="C:\\Temp\\robci_app_dataset_"+currentDateTime()+".csv";
	int 			rawlog_GZ_compress_level  = 1; 
	out_file.open( file_rawlog_output, rawlog_GZ_compress_level );

};
void RobcioLocalizationPF:: closeFileExport(){	
	out_file.close();
};

string  RobcioLocalizationPF:: currentDateTime() {
	time_t     now = time(0);
	struct tm  tstruct;
	char       buf[80];
	tstruct = *localtime(&now);
	// Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
	// for more information about date/time format
	strftime(buf, sizeof(buf), "%Y-%m-%d-%H-%M-%S", &tstruct);

	return buf;
};