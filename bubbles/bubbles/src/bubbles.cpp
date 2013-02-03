#include <cstdlib>
#include <cstddef>
#include <cmath>
#include <iostream>
#include <ios>
#include <complex>
#include <valarray>
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
//FOR SHADER
#include <OpenSG/OSGShaderProgramChunk.h>
#include <OpenSG/OSGShaderProgram.h>
//FOR COLOR
#include <OpenSG/OSGMaterialGroup.h>
#include <OpenSG/OSGImage.h>
#include <OpenSG/OSGSimpleTexturedMaterial.h>
//FOR BACKGROUND
#include <OpenSG/OSGSkyBackground.h>
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

OSG_USING_NAMESPACE // activate the OpenSG namespace


//------------------------------------------------------------------------------
// Global Variables
//------------------------------------------------------------------------------
#define VELOCITY		(500) // (100) = eigener Laptop, (500) = Linux Cave
OSGCSM::CAVEConfig cfg;
OSGCSM::CAVESceneManager *mgr = nullptr; // the CaveSceneManager to manage applications
vrpn_Tracker_Remote* tracker =  nullptr; // tracking
vrpn_Button_Remote* button = nullptr;
vrpn_Analog_Remote* analog = nullptr;
ComponentTransformRecPtr headCT;
ComponentTransformRecPtr directionCT;
NodeRecPtr scene;
std::list<NodeRecPtr> nodesToRemove;
std::map <NodeRecPtr, std::tuple<OSG::Vec3f,float>> nodePositions;
//Audio
#define NUM_CHANNELS		(1)
#define SAMPLE_RATE			(44100)
#define FRAMES_PER_BUFFER	(1024)
#define SAMPLE_FORMAT		(paFloat32)
#define inDevNum			(1)
PaStreamParameters inputParameters;
PaStream *stream; 
PaError err;
// FFT
typedef std::complex<double> Complex;
typedef std::valarray<Complex> CArray;
// Modi
std::tuple<bool,float> blowing = std::tuple<bool,float>(false,0.0);
float frequency = 0;
bool mode = false; // False = create Bubbles, true = fft
const double PI = 3.141592653589793238460;
 


//------------------------------------------------------------------------------
// Forward Declarations
//------------------------------------------------------------------------------
// forward declaration so we can have the interesting parts upfront
void setupGLUT(int *argc, char *argv[]);
void print_tracker();
NodeTransitPtr createBubble(float radius);
void keyboard(unsigned char k, int x, int y);
void cleanup(); //cleanup the used modules and databases

//------------------------------------------------------------------------------
// Helper Direction Node
//------------------------------------------------------------------------------

Vec3f getDirection(ComponentTransformRecPtr parentCT, ComponentTransformRecPtr childCT){
	Vec3f g = parentCT->getTranslation();
	Vec3f p = childCT->getTranslation();
	Quaternion q = parentCT->getRotation();
	Vec3f p2 = Vec3f(0,0,0);

	q.multVec(p * parentCT->getScale()[0], p2);
	p2 = p2 + g;
	return p2;
}

//------------------------------------------------------------------------------
// PortAudio
//------------------------------------------------------------------------------

 
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

	// creating bubbles
	if (!mode){
		for ( i=0; i<framesPerBuffer; i++ ){
			if (in[i] >0.4) {
				//printf(  "patestCallback: %f\n", in[i] );
				blowing = std::tuple<bool, float>(true, in[i]);
			} else {
				blowing = std::tuple<bool, float>(false, 0.0);
			}
		}
	}
	// move bubbles according to pitch
	else {

		// Apply Han window
		Complex test[FRAMES_PER_BUFFER];
		for ( i=0; i<framesPerBuffer; i++ ){
			test[i] = in[i] * (.5 * ( 1 - cos( 2 * PI * i / (FRAMES_PER_BUFFER-1.0) ) ));
		}
		CArray data(test, FRAMES_PER_BUFFER);

		// FFT
		fft(data);
 
		// Find the peak
		float maxVal = -1;
		int maxIndex = -1;
		for (int i = 0; i < FRAMES_PER_BUFFER ; ++i)
		{
			float x = sqrt(data[i].real() *data[i].real() + data[i].imag() *data[i].imag() );
			if( x > maxVal){
				maxVal = x;
				maxIndex = i;
			}
		}

		frequency = maxIndex*(SAMPLE_RATE/FRAMES_PER_BUFFER);
		//std::cout <<  frequency << " " << std::endl;
	}

    return 0;
}

int setupPortAudio() {

	// Initialize PortAudio
	err = Pa_Initialize();
	if( err != paNoError ) printf(  "PortAudio Pa_Initialize() error: %s\n", Pa_GetErrorText( err ) );	

	int numDevices;
	numDevices = Pa_GetDeviceCount();
	if( numDevices < 0 )
		{
			printf( "ERROR: Pa_CountDevices returned 0x%x\n", numDevices );
			err = numDevices;
		}
	else {
		//const   PaDeviceInfo *deviceInfo;
		//for( int i=0; i<numDevices; i++ )
			//{
			//deviceInfo = Pa_GetDeviceInfo( i );
			//std::cout << " device " << i <<": " << deviceInfo->name << std::endl;
			
			inputParameters.channelCount = NUM_CHANNELS;
			inputParameters.device = inDevNum;
			inputParameters.hostApiSpecificStreamInfo = NULL;
			inputParameters.sampleFormat =SAMPLE_FORMAT;
			inputParameters.suggestedLatency = Pa_GetDeviceInfo(inDevNum)->defaultLowInputLatency ;
			inputParameters.hostApiSpecificStreamInfo = NULL; //See you specific host's API docs for info on using this field


			err = Pa_IsFormatSupported(&inputParameters, NULL , SAMPLE_RATE );
			if( err == paFormatIsSupported )
			{
			     printf("paFormatIsSupported.\n");
			}
			else
			{
			   printf("Too Bad. paFormatIs NOT Supported \n");
			}

			err = Pa_OpenStream( 
				&stream,
                &inputParameters,	// one input channel 
                NULL,				// no output 
                SAMPLE_RATE,		// sample rate 
				FRAMES_PER_BUFFER,	// frames per buffer
				paNoFlag,			// flags for clipping etc
                patestCallback,		// this is your callback function 
                NULL);				// no data to pass 
			if( err != paNoError ) printf(  "PortAudio Pa_OpenStream() error: %s\n", Pa_GetErrorText( err ) );
		//}
	}

	 /* Open an audio I/O stream. */
    /*err = Pa_OpenDefaultStream( &stream,
                                NUM_CHANNELS,		// one input channel 
                                0,					// no output 
                                SAMPLE_FORMAT,		// sample format 
                                SAMPLE_RATE,		// sample rate 
								FRAMES_PER_BUFFER,	// frames per buffer
                                patestCallback,		// this is your callback function 
                                NULL);				// no data to pass 
    if( err != paNoError ) printf(  "PortAudio Pa_OpenDefaultStream() error: %s\n", Pa_GetErrorText( err ) ); */

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

auto head_orientation = Quaternion(Vec3f(0.f, 1.f, 0.f), osgDegree2Rad(180));
auto head_position = Vec3f(0.f, 170.f, 200.f);	// a 1.7m Person 2m in front of the scene

void VRPN_CALLBACK callback_head_tracker(void* userData, const vrpn_TRACKERCB tracker)
{
	head_orientation = Quaternion(tracker.quat[0], tracker.quat[1], tracker.quat[2], tracker.quat[3]);
	head_position = Vec3f(scale_tracker2cm(Vec3d(tracker.pos)));
	headCT->setRotation(head_orientation);
	headCT->setTranslation(head_position);
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
	if (button.button == 0 && button.state == 1){
		scene->addChild(createBubble(0.5));
		print_tracker();
	}
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
	std::cout << "Head position: " << head_position << " orientation: " << head_orientation << " orientation(Deg): " << osgRad2Degree(acos(head_orientation.w())*2)  << '\n';
	std::cout << "Wand position: " << wand_position << " orientation: " << wand_orientation << '\n';
	std::cout << "Analog: " << analog_values << '\n';

}

//------------------------------------------------------------------------------
// buildScene
//------------------------------------------------------------------------------
NodeTransitPtr createBubble(float radius) {
	//printf(  "createBubble \n" );

	//NodeRecPtr bubble = makeSphere(2,3);
	GeometryRecPtr bubbleGeo = makeSphereGeo(2, (1/radius)*(1/radius)*(1/radius));
	NodeRecPtr bubble = Node::create();
	bubble->setCore(bubbleGeo);

	SimpleTexturedMaterialRecPtr tex = SimpleTexturedMaterial::create();
	ImageRecPtr image = Image::create();
	if(!image->read("images/rainbow.png"))
    {
        fprintf(stderr, "Couldn't read texture 'rainbow.png'\n");
    }
	tex->setImage(image);
	tex->setTransparency(0.8);
	bubbleGeo->setMaterial(tex);

    // component transform ************************************
    
    ComponentTransformRecPtr ct = ComponentTransform::create();
	Vec3f direction = getDirection(headCT, directionCT) - headCT->getTranslation();
	Vec3f offset = Vec3f(0,-10,0);
	ct->setTranslation(head_position  + direction*30  + offset );

	NodeRecPtr bubbleTrans = Node::create();
	bubbleTrans->setCore(ct);
	bubbleTrans->addChild(bubble);
   
    // end component transform ********************************

	std::string nodeName = "bubbleTrans:" + std::to_string(direction.x()) + ":" + std::to_string(direction.y()) + ":" + std::to_string(direction.z())+ ":" + std::to_string(radius)+ ":" + std::to_string(glutGet(GLUT_ELAPSED_TIME));

	setName(bubbleGeo, "bubbleGeo");
	setName(bubble, "bubble");
	setName(bubbleTrans, nodeName);

	return NodeTransitPtr(bubbleTrans);
}

NodeTransitPtr buildScene()
{
	NodeRecPtr scene = Node::create();
	setName(scene, "scene");
	scene->setCore(Group::create());

	//Make Ground
	GeometryRecPtr groundGeo = makePlaneGeo(270.f, 270.f, 1, 1);
	SimpleMaterialRecPtr mat = SimpleMaterial::create();
	mat->setAmbient(Color3f(0.4f, 0.f, 0.f));
	groundGeo->setMaterial(mat);
	NodeRecPtr ground = Node::create();
	ground->setCore(groundGeo);
	setName(ground, "ground");
	//But it to our Feet
	ComponentTransformRecPtr ct = ComponentTransform::create();
	ct->setRotation(Quaternion(Vec3f(1,0,0),osgDegree2Rad(90)));
	NodeRecPtr groundTrans = Node::create();
	setName(groundTrans, "groundTrans");
	groundTrans->setCore(ct);
	groundTrans->addChild(ground);

	//Make Head
	NodeRecPtr head = Node::create();
	headCT = ComponentTransform::create();
	head->setCore(headCT);
	setName(head,"head");

	//Make Direction
	Vec3f direction_init = Vec3f(0,0,1);
	NodeRecPtr direction = Node::create();
	directionCT = ComponentTransform::create();
	directionCT->setTranslation((getDirection(headCT, directionCT) - direction_init) - headCT->getTranslation());
	direction->setCore(directionCT);
	setName(direction, "direction");

	//Add it to Root
	head->addChild(direction);
	scene->addChild(groundTrans);
	scene->addChild(head);

	// Skybox
	int scaleSkybox = 1;

	//skydome_day.WRL
	/*NodeRecPtr skybox = SceneFileHandler::the()->read("images/skydome/skydome_day.WRL");
	setName(skybox, "skybox");
	ComponentTransformRecPtr skyboxCT = ComponentTransform::create();
	skyboxCT->setTranslation(Vec3f(0,0,0));
	skyboxCT->setScale(Vec3f(scaleSkybox,scaleSkybox,scaleSkybox));

	
	skybox->setCore(skyboxCT);
	// Create skybox Transform
	NodeRecPtr skyboxTrans = Node::create();
	setName(skyboxTrans, "skyboxTrans");

	//NodeRecPtr skyboxTrans = makeNodeFor(skyboxCT);
	skyboxTrans->addChild(skybox);
	scene->addChild(skyboxTrans);

	// Read floor texture
	/*ImageRecPtr imageSkybox = Image::create();
	imageSkybox->read("images/skydome/maps/Skymap_cambridge_gen2_square.jpg");

	// Create floor texture from floor image
	SimpleTexturedMaterialRecPtr texSkybox = SimpleTexturedMaterial::create();
	texSkybox->setImage(imageSkybox);*/

	return NodeTransitPtr(scene);
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
}

void detectCollisions(OSG::Node * const currentNode, Vec3f currentPosition, float currentRadius){

	// Collision with wand
	//wand_position = head_position;
	float xd = currentPosition.x() - wand_position.x();
	float yd = currentPosition.y() - wand_position.y();
	float zd = currentPosition.z() - wand_position.z();

	float distance = std::sqrt(xd*xd + yd*yd + zd*zd);

	if( (std::abs(distance)) < ( (1/currentRadius)*(1/currentRadius)*(1/currentRadius) + 10)){
		//std::cout << "wand collisionDetected: \n";
		nodesToRemove.push_back(currentNode);
		return;
	}

	// Collision with other bubbles
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
				float currentR = (1/currentRadius)*(1/currentRadius)*(1/currentRadius);
				float r = (1/radius)*(1/radius)*(1/radius);
				if( ((std::abs(distance)) < ( currentR + r)) && ((std::abs(distance)) > ( std::max(currentR ,r)))  ){

					//std::cout << "collisionDetected: \n";

					std::vector<std::string> properties_currentNode = getProperties(getName(currentNode));
					std::vector<std::string> properties_nodeToCompare = getProperties(getName(nodeToCompare));
					// currentNode < nodeToCompare
					if(std::stof( properties_currentNode[4]) < std::stof( properties_nodeToCompare[4]) ){
						std::string nodeName = "bubbleTrans:" + properties_nodeToCompare[1] + ":" + properties_nodeToCompare[2] + ":" + properties_nodeToCompare[3] + ":" + properties_nodeToCompare[4] + ":" + properties_currentNode[5];
						setName(currentNode, nodeName);
					}
					// currentNode > nodeToCompare
					else {
						std::string nodeName = "bubbleTrans:" + properties_currentNode[1] + ":" + properties_currentNode[2] + ":" + properties_currentNode[3] + ":" + properties_currentNode[4] + ":" + properties_nodeToCompare[5];
						setName(nodeToCompare, nodeName);
					}

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

		Real32 time = glutGet(GLUT_ELAPSED_TIME);
		std::vector<std::string> properties = getProperties(name);
		Real32 ttl = std::stof(properties[5]);
		float v = std::stof( properties[4] );

		if( (time-ttl) < (VELOCITY *100 *v) ) {
			float x = std::stof( properties[1] );
			float y = std::stof( properties[2] );
			float z = std::stof( properties[3] );
			
			ComponentTransformRecPtr bt = dynamic_cast<ComponentTransform*>(node->getCore());

			Vec3f vec = bt->getTranslation();
			Vec3f direction = OSG::Vec3f( x , y - ((VELOCITY*v)/200000)  ,z );
			Vec3f vec2 = vec + direction * (VELOCITY*v)/2000 ;

			bt->setTranslation( vec2 ); //MOVE

			nodePositions[node] = std::tuple<OSG::Vec3f,float>(vec, v);
			detectCollisions(node, vec, v);

			// ground arrived
			if ( vec.y() < v/2 ) {
				direction = OSG::Vec3f( 0 , 0 , 0 );
			}

			// update ttl in nodeName
			std::string nodeName = "bubbleTrans:" + std::to_string(direction.x()) + ":" + std::to_string(direction.y()) + ":" + std::to_string(direction.z())+ ":" + std::to_string(v)+ ":" + std::to_string(ttl);
			setName(node, nodeName);

			//std::cout << "nodePositions Size: " << nodePositions.size()  <<" \n";
		}
		else {
			nodesToRemove.push_back(node);
		}
    }
	else if (std::strcmp(name, "bubbleGroup") == 0) {
		if ( node->getNChildren() == 0 ) nodesToRemove.push_back(node);
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

OSG::Action::ResultE moveBubbles(OSG::Node * const node) {
	const char *name = getName(node);

	if(isBubbleTrans(name))
    {
		Real32 time = glutGet(GLUT_ELAPSED_TIME);
		std::vector<std::string> properties = getProperties(name);
		float v = std::stof( properties[4] );
		float x = 0;
		float y = -1 ;
		if (frequency > 100 && frequency < 1000) {
			y = ((std::log10(frequency)) - (std::log10(330)) ) *100 ;  // 330 = e
		}
		float z = 0;

		ComponentTransformRecPtr bt = dynamic_cast<ComponentTransform*>(node->getCore());

		Vec3f vec = bt->getTranslation();
		Vec3f direction = OSG::Vec3f( x , y * ((VELOCITY*v)/20000) ,z );
		Vec3f vec2 = vec + direction;

		bt->setTranslation( vec2 ); //MOVE

		// ground arrived
		if ( vec.y() <v/2 ) {
			direction = OSG::Vec3f( 0 , 0 , 0 );
		}
    } 

    return OSG::Action::Continue; 
}

void setupGLUT(int *argc, char *argv[])
{
	glutInit(argc, argv);
	glutInitDisplayMode(GLUT_RGB  |GLUT_DEPTH | GLUT_DOUBLE);
	glutCreateWindow("Bubbles");
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
		if(!mode) {
			if ( std::get<0>(blowing) ) scene->addChild(createBubble( std::get<1>(blowing)));	// create Bubble
			traverse(scene, enter);			// traverse Scenegraph
			removeOldBubbles();
		}
		else {
			traverse(scene, moveBubbles);		
		}
		commitChanges();
		mgr->redraw();
		// the changelist should be cleared - else things could be copied multiple times
		OSG::Thread::getCurrentChangeList()->clear();
	});
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

		// Start PortAudio Stream
		err = Pa_StartStream( stream ); 
		if( err != paNoError ) printf(  "PortAudio Pa_StartStream() error: %s\n", Pa_GetErrorText( err ) );


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
// CLEANUP
//------------------------------------------------------------------------------

void cleanup()
{
	delete mgr;
	delete tracker;
	delete button;
	delete analog;
	scene = NULL;
	headCT = NULL;
	directionCT = NULL;
	nodePositions.clear();
	nodesToRemove.clear();
	err = Pa_Terminate();
	if( err != paNoError ) printf(  "PortAudio Pa_Terminate() error: %s\n", Pa_GetErrorText( err ) );

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
			err = Pa_StartStream( stream ); 
			if( err != paNoError ) printf(  "PortAudio Pa_StartStream() error: %s\n", Pa_GetErrorText( err ) );
			//Pa_IsStreamActive
		break;
		case 'g':
			head_orientation = Quaternion(head_orientation.x(),head_orientation.y(),head_orientation.z()-0.05f,head_orientation.w());
			break;
		case 'v':
			head_orientation = Quaternion(head_orientation.x(),head_orientation.y(),head_orientation.z(), cos((acos(head_orientation.w())*2 + PI/36)/2));
			break;
		case 'b':
			head_orientation = Quaternion(head_orientation.x(),head_orientation.y(),head_orientation.z()+0.05f,head_orientation.w());
			break;
		case 'n' : 
			head_orientation = Quaternion(head_orientation.x(),head_orientation.y(),head_orientation.z(), cos((acos(head_orientation.w())*2-PI/36)/2));
			break;
		case 'y':
			head_orientation = Quaternion(head_orientation.x()-0.05f,head_orientation.y(),head_orientation.z(),head_orientation.w());
			break;
		case 'x' : 
			head_orientation = Quaternion(head_orientation.x()+0.05f,head_orientation.y(),head_orientation.z(), head_orientation.w());
			break;
		case 'G':
			head_position = Vec3f(head_position.x(),head_position.y(),head_position.z()-5);
			break;
		case 'V':
			head_position = Vec3f(head_position.x()-5,head_position.y(),head_position.z());
			break;
		case 'B':
			head_position = Vec3f(head_position.x(),head_position.y(),head_position.z()+5);
			break;
		case 'N' : 
			head_position = Vec3f(head_position.x()+5,head_position.y(),head_position.z());
			break;
		case 's' : //stop
			// Stop PortAudio Stream
			err = Pa_AbortStream( stream ); // or Pa_StopStream( stream );
			if( err != paNoError ) printf(  "PortAudio Pa_StopStream() error: %s\n", Pa_GetErrorText( err ) );
			break;
		case 'm' : 
			mode = !mode;
			break;
		default:
			std::cout << "Key '" << k << "' ignored\n";
	}
}