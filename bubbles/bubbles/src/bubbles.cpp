#include <cstdlib>
#include <cstddef>
#include <cmath>
#include <iostream>
#include <ios>
#include <boost/algorithm/string.hpp>
//OpenSG includes
#include <OpenSG/OSGGLUT.h>
#include <OpenSG/OSGConfig.h>
#include <OpenSG/OSGMultiDisplayWindow.h>
#include <OpenSG/OSGSceneFileHandler.h>
#include <OpenSG/OSGGLUTWindow.h>
//FOR CREATING CHILD NODES
#include <OpenSG/OSGSimpleGeometry.h>
#include <OpenSG/OSGSimpleGeometry.h>
#include <OpenSG/OSGNameAttachment.h>
//FOR TRANSFORMATIONS
#include <OpenSG/OSGComponentTransform.h>
//FOR COLOR
#include <OpenSG/OSGMaterialGroup.h>
#include <OpenSG/OSGImage.h>
#include <OpenSG/OSGSimpleTexturedMaterial.h>
//FOR CAVESCEBEMANAGER
#include <OSGCSM/OSGCAVESceneManager.h>
#include <OSGCSM/OSGCAVEConfig.h>
#include <OSGCSM/appctrl.h>
//FOR TRACKING
#include <vrpn_Tracker.h>
#include <vrpn_Button.h>
#include <vrpn_Analog.h>
//For AUDIO
#include "portaudio.h"
//using namespace std;

OSG_USING_NAMESPACE // activate the OpenSG namespace

#include <complex>
#include <iostream>
#include <valarray>

 
const double PI = 3.141592653589793238460;
 
typedef std::complex<double> Complex;
typedef std::valarray<Complex> CArray;



 
// Cooley–Tukey FFT (in-place, divide-and-conquer)
// Higher memory requirements and redundancy although more intuitive
void fft(CArray& x)
{
    const size_t N = x.size();
    if (N <= 1) return;
 
    // divide
    CArray even = x[std::slice(0, N/2, 2)];
    CArray  odd = x[std::slice(1, N/2, 2)];
 
    // conquer
    fft(even);
    fft(odd);
 
    // combine
    for (size_t k = 0; k < N/2; ++k)
    {
        Complex t = std::polar(1.0, -2 * PI * k / N) * odd[k];
        x[k    ] = even[k] + t;
        x[k+N/2] = even[k] - t;
    }
}
 
// Cooley-Tukey FFT (in-place, breadth-first, decimation-in-frequency)
// Better optimized but less intuitive
/*void fft(CArray &x)
{
	// DFT
	unsigned int N = x.size(), k = N, n;
	double thetaT = 3.14159265358979323846264338328L / N;
	Complex phiT = Complex(cos(thetaT), sin(thetaT)), T;
	while (k > 1)
	{
		n = k;
		k >>= 1;
		phiT = phiT * phiT;
		T = 1.0L;
		for (unsigned int l = 0; l < k; l++)
		{
			for (unsigned int a = l; a < N; a += n)
			{
				unsigned int b = a + k;
				Complex t = x[a] - x[b];
				x[a] += x[b];
				x[b] = t * T;
			}
			T *= phiT;
		}
	}
	// Decimate
	unsigned int m = (unsigned int)log2(N);
	for (unsigned int a = 0; a < N; a++)
	{
		unsigned int b = a;
		// Reverse bits
		b = (((b & 0xaaaaaaaa) >> 1) | ((b & 0x55555555) << 1));
		b = (((b & 0xcccccccc) >> 2) | ((b & 0x33333333) << 2));
		b = (((b & 0xf0f0f0f0) >> 4) | ((b & 0x0f0f0f0f) << 4));
		b = (((b & 0xff00ff00) >> 8) | ((b & 0x00ff00ff) << 8));
		b = ((b >> 16) | (b << 16)) >> (32 - m);
		if (b > a)
		{
			Complex t = x[a];
			x[a] = x[b];
			x[b] = t;
		}
	}
	// Normalize
	Complex f = 1.0 / sqrt(N);
	for (unsigned int i = 0; i < N; i++)
		x[i] *= f;
}*/


//------------------------------------------------------------------------------
// Global Variables
//------------------------------------------------------------------------------
OSGCSM::CAVEConfig cfg;
OSGCSM::CAVESceneManager *mgr = nullptr; // the CaveSceneManager to manage applications
vrpn_Tracker_Remote* tracker =  nullptr; // tracking
vrpn_Button_Remote* button = nullptr;
vrpn_Analog_Remote* analog = nullptr;
NodeRecPtr scene;
NodeRecPtr buuble;
std::list<NodeRecPtr> nodesToRemove;
std::map <NodeRecPtr, std::tuple<OSG::Vec3f,float>> nodePositions;
//Calculating Perspective
Vec3f direction =  Vec3f(0,0,0);
//Audio
#define NUM_CHANNELS		(1)
#define SAMPLE_RATE			(44100)
#define FRAMES_PER_BUFFER	(1024)
#define SAMPLE_FORMAT		(paFloat32)
#define BUBBLES_PER_SECOND	(1)
std::tuple<boolean,float> blowing = std::tuple<boolean,float>(false,0.0);
PaStream *stream;
PaError err;
int counter = 0;

//------------------------------------------------------------------------------
// Forward Declarations
//------------------------------------------------------------------------------
// forward declaration so we can have the interesting parts upfront
void setupGLUT(int *argc, char *argv[]);
void print_tracker();
NodeRecPtr createBubble(float radius);
void cleanup(); //cleanup the used modules and databases


//------------------------------------------------------------------------------
// PortAudio
//------------------------------------------------------------------------------


static int patestCallback( const void *inputBuffer, void *outputBuffer,
                           unsigned long framesPerBuffer,
                           const PaStreamCallbackTimeInfo* timeInfo,
                           PaStreamCallbackFlags statusFlags,
                           void *userData )
{
	float *in = (float*)inputBuffer;
	(void) outputBuffer;  /* Prevent unused variable warning. */
	(void) userData;
	unsigned int i;

	for ( i=0; i<framesPerBuffer; i++ ){
		if (in[i] >0.5) {
			//printf(  "patestCallback: %f\n", in[i] );
			blowing = std::tuple<boolean, float>(true, in[i]);
		} else {
			blowing = std::tuple<boolean, float>(false, 0.0);
		}
	}

    return 0;
}

int setupPortAudio() {

	// Initialize PortAudio
	err = Pa_Initialize();
	if( err != paNoError ) printf(  "PortAudio Pa_Initialize() error: %s\n", Pa_GetErrorText( err ) );	

    /* Open an audio I/O stream. */
    err = Pa_OpenDefaultStream( &stream,
                                NUM_CHANNELS,		/* one input channel */
                                0,					/* no output */
                                SAMPLE_FORMAT,		/* sample format */
                                SAMPLE_RATE,		/* sample rate */
								FRAMES_PER_BUFFER,	/* frames per buffer*/
                                patestCallback,		/* this is your callback function */
                                NULL);				/* no data to pass */
    if( err != paNoError ) printf(  "PortAudio Pa_OpenDefaultStream() error: %s\n", Pa_GetErrorText( err ) );

	return err;
}


//------------------------------------------------------------------------------
// Tracking
//------------------------------------------------------------------------------

template<typename T>
T scale_tracker2cm(const T& value)
{
	static const float scale = OSGCSM::convert_length(cfg.getUnits(), 1.f, OSGCSM::CAVEConfig::CAVEUnitCentimeters);
	return value * scale;
}

auto head_orientation = Quaternion(Vec3f(0.f, 1.f, 0.5f), osgDegree2Rad(180));
auto head_position = Vec3f(0.f, 170.f, 200.f);	// a 1.7m Person 2m in front of the scene

void VRPN_CALLBACK callback_head_tracker(void* userData, const vrpn_TRACKERCB tracker)
{
	head_orientation = Quaternion(tracker.quat[0], tracker.quat[1], tracker.quat[2], tracker.quat[3]);
	head_position = Vec3f(scale_tracker2cm(Vec3d(tracker.pos)));
}

auto wand_orientation = Quaternion();
auto wand_position = Vec3f();
void VRPN_CALLBACK callback_wand_tracker(void* userData, const vrpn_TRACKERCB tracker)
{
	wand_orientation = Quaternion(tracker.quat[0], tracker.quat[1], tracker.quat[2], tracker.quat[3]);
	wand_position = Vec3f(scale_tracker2cm(Vec3d(tracker.pos)));
}

auto analog_values = Vec3f();
void VRPN_CALLBACK callback_analog(void* userData, const vrpn_ANALOGCB analog)
{
	if (analog.num_channel >= 2)
		analog_values = Vec3f(analog.channel[0], 0, -analog.channel[1]);
}

void VRPN_CALLBACK callback_button(void* userData, const vrpn_BUTTONCB button)
{
	if (button.button == 0 && button.state == 1)
		print_tracker();
}

void InitTracker(OSGCSM::CAVEConfig &cfg)
{
	try
	{
		const char* const vrpn_name = "DTrack@localhost";
		tracker = new vrpn_Tracker_Remote(vrpn_name);
		tracker->shutup = true;
		tracker->register_change_handler(NULL, callback_head_tracker, cfg.getSensorIDHead());
		tracker->register_change_handler(NULL, callback_wand_tracker, cfg.getSensorIDController());
		button = new vrpn_Button_Remote(vrpn_name);
		button->shutup = true;
		button->register_change_handler(nullptr, callback_button);
		analog = new vrpn_Analog_Remote(vrpn_name);
		analog->shutup = true;
		analog->register_change_handler(NULL, callback_analog);
	}
	catch(const std::exception& e) 
	{
		std::cout << "ERROR: " << e.what() << '\n';
		return;
	}
}

void check_tracker()
{
	tracker->mainloop();
	button->mainloop();
	analog->mainloop();
}

void print_tracker()
{
	std::cout << "Head position: " << head_position << " orientation: " << head_orientation << " orientation(Deg): " << head_orientation.w() << '\n';
	std::cout << "Wand position: " << wand_position << " orientation: " << wand_orientation << '\n';
	std::cout << "Analog: " << analog_values << '\n';

}

//------------------------------------------------------------------------------
// buildScene
//------------------------------------------------------------------------------
NodeRecPtr createBubble(float radius) {
	//printf(  "createBubble \n" );

	//NodeRecPtr bubble = makeSphere(2,3);
	GeometryRecPtr bubbleGeo = makeSphereGeo(2, (1/radius)*(1/radius)*(1/radius));
	NodeRecPtr bubble = Node::create();
	bubble->setCore(bubbleGeo);

    // component transform ************************************
    
    //this one is declared globally
    ComponentTransformRecPtr ct = ComponentTransform::create();

	
	try{

		// calculate orthogonal vector on head_orientation vector (always in y,z plane and in +z direction)
		Vec3f v = Vec3f(1,0,0).cross(Vec3f(head_orientation.x(), head_orientation.y(), head_orientation.z()));
		// turn vector the same angle the orientation vector is turned around the y axis
		head_orientation.multVec(v, direction);

		// + mgr->getTranslation() ?
		ct->setTranslation(head_position + direction*30 );

		print_tracker();
		std::cout << "Orthogonal vector: " << v << " direction: " << direction << '\n';

	}
	catch(const std::exception& e)
	{
		printf("calculating head_position failed \n");
	}

	NodeRecPtr bubbleTrans = Node::create();
	bubbleTrans->setCore(ct);
	bubbleTrans->addChild(bubble);
   
    // end component transform ********************************
	std::string nodeName = "bubbleTrans:" + std::to_string(direction.x()) + ":" + std::to_string(direction.y()) + ":" + std::to_string(direction.z())+ ":" + std::to_string(radius)+ ":" + std::to_string(glutGet(GLUT_ELAPSED_TIME));

	setName(bubbleGeo, "bubbleGeo");
	setName(bubble, "bubble");
	setName(bubbleTrans, nodeName);
	
	return bubbleTrans;
}

NodeTransitPtr buildScene()
{
	NodeRecPtr scene = Node::create();
	setName(scene, "scene");
	scene->setCore(Group::create());

	//Make Ground
	NodeRecPtr ground = makePlane(200.f, 200.f, 1, 1);
	setName(ground, "ground");
	//But it to our Feet
	ComponentTransformRecPtr ct = ComponentTransform::create();
	setName(ct, "ct");
	//ct->setTranslation(Vec3f(0,-2,0));
	ct->setRotation(Quaternion(Vec3f(1,0,0),osgDegree2Rad(90)));
	NodeRecPtr groundTrans = Node::create();
	setName(groundTrans, "groundTrans");
	groundTrans->setCore(ct);
	groundTrans->addChild(ground);
	//Add it to Root
	scene->addChild(groundTrans);

	return NodeTransitPtr(scene);
}

//------------------------------------------------------------------------------
// Keyboard Input
//------------------------------------------------------------------------------

void keyboard(unsigned char k, int x, int y)
{
	Real32 ed;
	switch(k)
	{
		case 'q':
		case 27: 
			cleanup();
			exit(EXIT_SUCCESS);
			break;
		case 'e':
			ed = mgr->getEyeSeparation() * .9f;
			std::cout << "Eye distance: " << ed << '\n';
			mgr->setEyeSeparation(ed);
			break;
		case 'E':
			ed = mgr->getEyeSeparation() * 1.1f;
			std::cout << "Eye distance: " << ed << '\n';
			mgr->setEyeSeparation(ed);
			break;
		case 'h':
			cfg.setFollowHead(!cfg.getFollowHead());
			std::cout << "following head: " << std::boolalpha << cfg.getFollowHead() << '\n';
			break;
		case 'i':
			print_tracker();
			break;
		case 'c' : // create
			// Start PortAudio Stream
			//scene->addChild(createBubble());
			err = Pa_StartStream( stream ); 
			if( err != paNoError ) printf(  "PortAudio Pa_StartStream() error: %s\n", Pa_GetErrorText( err ) );
			//Pa_IsStreamActive
		break;
		case 's' : //stop
			//myfile.close();
			// Stop PortAudio Stream
			err = Pa_AbortStream( stream ); // or Pa_StopStream( stream );
			if( err != paNoError ) printf(  "PortAudio Pa_StopStream() error: %s\n", Pa_GetErrorText( err ) );	 
		break;
		default:
			std::cout << "Key '" << k << "' ignored\n";
	}
}

//------------------------------------------------------------------------------
// MAIN
//------------------------------------------------------------------------------

int main(int argc, char **argv)
{
#if WIN32
	OSG::preloadSharedObject("OSGFileIO");
	OSG::preloadSharedObject("OSGImageFileIO");
#endif
	try
	{

		bool cfgIsSet = false;
		scene = nullptr;

		// ChangeList needs to be set for OpenSG 1.4
		ChangeList::setReadWriteDefault();
		osgInit(argc,argv);

		// evaluate intial params
		for(int a=1 ; a<argc ; ++a)
		{
			if( argv[a][0] == '-' )
			{
				if ( strcmp(argv[a],"-f") == 0 ) 
				{
					char* cfgFile = argv[a][2] ? &argv[a][2] : &argv[++a][0];
					if (!cfg.loadFile(cfgFile)) 
					{
						std::cout << "ERROR: could not load config file '" << cfgFile << "'\n";
						return EXIT_FAILURE;
					}
					cfgIsSet = true;
				}
			} else {
				std::cout << "Loading scene file '" << argv[a] << "'\n";
				scene = SceneFileHandler::the()->read(argv[a], NULL);
			}
		}

		// load the CAVE setup config file if it was not loaded already:
		if (!cfgIsSet) 
		{
			const char* const default_config_filename = "config/mono.csm";
			if (!cfg.loadFile(default_config_filename)) 
			{
				std::cout << "ERROR: could not load default config file '" << default_config_filename << "'\n";
				return EXIT_FAILURE;
			}
		}

		cfg.printConfig();

		// start servers for video rendering
		if ( startServers(cfg) < 0 ) 
		{
			std::cout << "ERROR: Failed to start servers\n";
			return EXIT_FAILURE;
		}

		setupGLUT(&argc, argv);

		InitTracker(cfg);

		setupPortAudio();


		MultiDisplayWindowRefPtr mwin = createAppWindow(cfg, cfg.getBroadcastaddress());

		if (!scene) 
			scene = buildScene();
		commitChanges();


		mgr = new OSGCSM::CAVESceneManager(&cfg);
		mgr->setWindow(mwin );
		mgr->setRoot(scene);
		mgr->showAll();
		mgr->getWindow()->init();
		mgr->turnWandOff();
	}
	catch(const std::exception& e)
	{
		std::cout << "ERROR: " << e.what() << '\n';
		return EXIT_FAILURE;
	}

	glutMainLoop();
}

//------------------------------------------------------------------------------
// GLUT
//------------------------------------------------------------------------------

bool isBubbleTrans(const char *str)
{
	const char *pre = "bubbleTrans";
    size_t lenpre = strlen(pre),
           lenstr = strlen(str);
    return lenstr < lenpre ? false : strncmp(pre, str, lenpre) == 0;
}

std::vector<std::string> getProperties(const char *str)
{
	std::vector<std::string> strs;
	boost::split(strs, str , boost::is_any_of(":"));

	return  strs;
	//TODO Fehlerbehandlung
}

void detectCollisions(OSG::Node * const currentNode, Vec3f currentPosition, float currentRadius){

	if(!nodePositions.empty()){
		for(std::map<NodeRecPtr, std::tuple<OSG::Vec3f, float>>::iterator list_iter = nodePositions.begin();
				list_iter != nodePositions.end(); list_iter++)
		{
			NodeRecPtr nodeToCompare = list_iter->first;
			if(currentNode != nodeToCompare){
				Vec3f position = std::get<0>(list_iter->second);
				float radius = std::get<1>(list_iter->second); 

				float xd = currentPosition.x() - position.x();
				float yd = currentPosition.y() - position.y();
				float zd = currentPosition.z() - position.z();

				float distance = std::sqrt(xd*xd + yd*yd + zd*zd);
				if( (std::abs(distance)) > ( (1/currentRadius)*(1/currentRadius)*(1/currentRadius) + (1/radius)*(1/radius)*(1/radius)) ){
					std::cout << "collisionDetected: \n";
				}
			}
		}
	}
}

OSG::Action::ResultE enter(OSG::Node * const node)
{
	const char *name = getName(node);

	if(isBubbleTrans(name))
	//if(strcmp(getName(node),"bubbleTrans") == 0)
    {
        //move bubble
		//printf(  "moveBubble \n" );

        //ct->setTranslation(OSG::Vec3f(0,cos(time/2000.f)*100,200)); 

		Real32 time = glutGet(GLUT_ELAPSED_TIME);
		std::vector<std::string> properties = getProperties(name);
		Real32 ttl = std::stof(properties[5]);
		float v = std::stof( properties[4] );

		if( (time-ttl) < (10000*v) ) {
			float x = std::stof( properties[1] );
			float y = std::stof( properties[2] );
			float z = std::stof( properties[3] );
			
			ComponentTransformRecPtr bt = dynamic_cast<ComponentTransform*>(node->getCore());

			Vec3f vec = bt->getTranslation();
			Vec3f direction = OSG::Vec3f( x , y - 0.001*v/2  ,z );
			Vec3f vec2 = vec + direction * ( v/20 );


			bt->setTranslation( vec2 ); //MOVE

			//bt->setTranslation(Vec3f(1,0.01*time,1));
			//bt->setRotation(Quaternion(Vec3f(1,0,0),osgDegree2Rad(270)+0.001f*time));
			//bt->setScale(Vec3f(0.001,0.001,0.001));

			nodePositions[node] = std::tuple<OSG::Vec3f,float>(vec, v);
			detectCollisions(node, vec, v);

			if ( vec.y() < v/2 ) {
				direction = OSG::Vec3f( 0 , 0 , 0 );
			}

			std::string nodeName = "bubbleTrans:" + std::to_string(direction.x()) + ":" + std::to_string(direction.y()) + ":" + std::to_string(direction.z())+ ":" + std::to_string(v)+ ":" + std::to_string(ttl);
			setName(node, nodeName);

			std::cout << "nodePositions Size: " << nodePositions.size()  <<" \n";
		}
		else {
			nodesToRemove.push_back(node);
		}
    } 

    return OSG::Action::Continue; 
}

void removeOldBubbles() {
	if(!nodesToRemove.empty()){
		for(std::list<NodeRecPtr>::iterator list_iter = nodesToRemove.begin(); 
		list_iter != nodesToRemove.end(); list_iter++)
			{
				//std::cout<<*list_iter<<endl;
				NodeRecPtr nodeToRemove = *list_iter;
				nodeToRemove->clearChildren();
				scene->subChild(nodeToRemove);

				nodePositions.erase(nodeToRemove);
			}
	}
	nodesToRemove.clear();
}

void setupGLUT(int *argc, char *argv[])
{
	glutInit(argc, argv);
	glutInitDisplayMode(GLUT_RGB  |GLUT_DEPTH | GLUT_DOUBLE);
	glutCreateWindow("OpenSG CSMDemo with VRPN API");
	glutDisplayFunc([]()
	{
		// black navigation window
		glClear(GL_COLOR_BUFFER_BIT);
		glutSwapBuffers();
	});
	glutReshapeFunc([](int w, int h)
	{
		mgr->resize(w, h);
		glutPostRedisplay();
	});
	glutKeyboardFunc(keyboard);
	glutIdleFunc([]()
	{
		// Tracker
		check_tracker();
		const auto speed = 1.f;
		mgr->setUserTransform(head_position, head_orientation);
		mgr->setTranslation(mgr->getTranslation() + speed * analog_values);
		// Bubbles
		if ( std::get<0>(blowing) ) scene->addChild(createBubble( std::get<1>(blowing)));	// create Bubble
		traverse(scene, enter);			// traverse Scenegraph
		removeOldBubbles();
		commitChanges();
		mgr->redraw();
		// the changelist should be cleared - else things could be copied multiple times
		OSG::Thread::getCurrentChangeList()->clear();
	});
}

//------------------------------------------------------------------------------
// CLEANUP
//------------------------------------------------------------------------------

void cleanup()
{
	delete mgr;
	delete tracker;
	delete button;
	delete analog;
	scene = NULL;
	err = Pa_Terminate();
	if( err != paNoError ) printf(  "PortAudio Pa_Terminate() error: %s\n", Pa_GetErrorText( err ) );

}