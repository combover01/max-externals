/**
	@file
	simplemsp~: a simple audio object for Max
	original by: jeremy bernstein, jeremy@bootsquad.com
	@ingroup examples
*/
#include <lsl_c.h>
#include <stdio.h>
// #include "buffer.h"
#include "ext_buffer.h"

#define MAXAPI_USE_MSCRT
#pragma comment(lib, "ws2_32.lib")
#define TID  HANDLE
#define MUTEX CRITICAL_SECTION

#include "ext.h"			// standard Max include, always required (except in Jitter)
#include "ext_obex.h"		// required for "new" style objects
#include "z_dsp.h"			// required for MSP objects

typedef struct ringBuff {
    double** buffer;
    int sampleRate;
    int nChannels;
    int buffLength;
    int writeIndex;
    int readIndex;
    bool isInit;
    bool buffFull;
} ringBuff;

// struct to represent the object's state
typedef struct _mydspobject {
	t_pxobject		ob;			// the object itself (t_pxobject in MSP instead of t_object)
	long			inputs;		// number of inlets (usually 1)
	int			outputs;	// number of outlets, set dynamically
	int			nchannels;
	int        connected;             // flag to say if we are connected or not
	int        ready;                 // flag to say if we have enough samples to start reading
	int        lag;                   // lag time to buffer up
	int       upsample;               // flag to upsample or not
	int       sr_msp;                  // max's sampling rate
	int       sr_lsl;
	double    sr_ratio;
    double    sr_ratio_inv;
	ringBuff	LSLbuffer;
	ringBuff outputRingBuff;
	ringBuff* lslBuffPtr;
	ringBuff* outputRingBuffPtr;
	int sizeOfLSLInputBuffer;
	int sizeOfOutputRingBuffer;
	double* d_internalOutputBuff;
	float* f_internalOutputBuff;
	double** d_newSampleFramesOutputBuff; 
	float* curLSLSamps;
	float* currentLSLSampleOut;
	float* previousLSLSampleOut;
	float* nextLSLSampleOut;

	t_buffer_ref* w_buf;
	t_bool w_buffer_modified;

	int index;

	lsl_inlet               lsl_inlet_obj;      // instantiation of the inlet class
	lsl_streaminfo          lsl_info_list[50];  // to contain list returned by continuous resolver
	int                     lsl_info_list_cnt;
	int                     which;
	lsl_continuous_resolver lsl_cr;
	lsl_channel_format_t    type;
	double                  lsl_pull_timeout;
	double                  lag_lsl;

	// needed for lsl pull sample
	float ts;
	int ec;

	int             stop_;
	int             can_launch_resolver;

	// NEW MAX THREADING OBJECTS!!
	t_systhread		x_systhread;						// thread reference
	t_systhread_mutex	x_mutex;							// mutual exclusion lock for threadsafety
	int				x_systhread_cancel;					// thread cancel flag
	// do we need a qelem??
	void* x_qelem;							// for message passing between threads
	int counter;

	bool lslBufferFullFlag;

	t_symbol* m_stream_name;
	t_symbol* m_stream_type;
	t_symbol* m_stream_id;
} t_mydspobject;


// method prototypes
void* mydsp_new(t_symbol* s, long argc, t_atom* argv);
int resample(double** inputBuffer, double** outputBuffer, int initSR, int dstSR, int inputBufferLength, int outputBufferLength, int nChannels);
void mydsp_free(t_mydspobject* x);
void mydsp_assist(t_mydspobject* x, void* b, long m, long a, char* s);
void mydsp_dsp64(t_mydspobject* x, t_object* dsp64, t_signal** sp);
void lsl_inlet_disconnect(t_mydspobject* x);
void destroy_info_list(t_mydspobject* x);
void lsl_inlet_connect_by_idx(t_mydspobject* x, t_atom_float f);
void lsl_inlet_list_all(t_mydspobject* x);
void post_info_list(t_mydspobject* x);
void lsl_inlet_resolve_by_property(t_mydspobject* x, t_symbol* s, long argc, t_atom* argv);
void lsl_inlet_list_by_property(t_mydspobject* x, t_symbol* s, long argc, t_atom* argv);
void mydsp_perform64(t_mydspobject* x, t_object* dsp64, double** ins, long numins, double** outs, long numouts, int sampleframes, long flags, void* userparam);

static int prop_resolve(t_mydspobject* x, long argc, t_atom* argv);

t_max_err mydsp_notify(t_mydspobject* x, t_symbol* s, t_symbol* msg, void* sender, void* data);

// global class pointer variable
static t_class* mydsp_class = NULL;

// NEW MAX THREADING CLASSES
void mydspthread_stop(t_mydspobject* x);
void mydspthread_cancel(t_mydspobject* x);
void* mydspthread_threadproc(t_mydspobject* x);

void mydsp_dblclick(t_mydspobject* x);

int resample(double** inputBuffer, double** outputBuffer, int initSR, int dstSR, int inputBufferLength, int outputBufferLength, int nChannels);

static t_symbol* ps_buffer_modified;

int curOutputIndex = 0;

//***********************************************************************************************

void InitBuff(ringBuff *buff, const int nChannel, const int buffLength)
{
    buff->nChannels = nChannel;
    buff->buffLength = buffLength * 2;
    post("current buffLength before multiplying: %i", buffLength);
    post("actual buff bufflength: %i", buff->buffLength);
    // https://stackoverflow.com/questions/29977084/calloc-a-two-dimensional-array
    // from vlad from moscow

    buff->buffer = malloc(nChannel * sizeof(double*));
    if (!buff->buffer) {                   /* if you allocate -- you must VALIDATE */
        perror("malloc-m");
        return NULL;
    }
    for (int i = 0; i < nChannel; i++) {
        buff->buffer[i] = calloc(buffLength * 2, sizeof(double));
        if (!buff->buffer[i]) {                   /* if you allocate -- you must VALIDATE */
            perror("calloc-m");
            return NULL;
        }
    }
//    we need readindex always behind writeindex by the delay amount
    buff->readIndex = 0;
    buff->writeIndex = buffLength;
    buff->buffFull = false;
    buff->isInit = true;
}

void writeBuff(t_mydspobject* x, ringBuff *buff, float* samples2write)
{
	systhread_mutex_lock(x->x_mutex);

    while (buff->writeIndex >= buff->buffLength)
    {
        buff->writeIndex = buff->writeIndex - buff->buffLength;
        post("writeIndex got wrapped back to %i", buff->writeIndex);
    }
    if (buff->writeIndex < 0){
        buff->writeIndex = buff->writeIndex + buff->buffLength;
        post("writeIndex got wrapped forward to %i", buff->writeIndex);
    }
    for (int i = 0; i < buff->nChannels; i++)
    {
        buff->buffer[i][buff->writeIndex] = samples2write[i];
    }
    post("current writeIndex: %i for buff w readIndex %i and bufflength %i", buff->writeIndex, buff->readIndex, buff->buffLength);
    buff->writeIndex = buff->writeIndex + 1;
    

    if (buff->writeIndex >= buff->buffLength) {
        buff->buffFull = true;
    }
    else {
        buff->buffFull = false;
    }
	systhread_mutex_unlock(x->x_mutex);

}
void writeBuff_with_index(t_mydspobject* x, ringBuff *buff, float* samples2write, int index)
{
	systhread_mutex_lock(x->x_mutex);

	buff->writeIndex = index;
    while (buff->writeIndex >= buff->buffLength)
    {
        buff->writeIndex = buff->writeIndex - buff->buffLength;
        post("writeIndex got wrapped back in writebuffwithindex to %i", buff->writeIndex);
    }
    if (buff->writeIndex < 0){
        buff->writeIndex = buff->writeIndex + buff->buffLength;
        post("writeIndex got wrapped forward in writebuffwithindex to %i", buff->writeIndex);
    }
    for (int i = 0; i < buff->nChannels; i++)
    {
        buff->buffer[i][buff->writeIndex] = samples2write[i];
    }
    post("current writeIndex in writebuffwithindex: %i for buff w readIndex %i and bufflength %i", buff->writeIndex, buff->readIndex, buff->buffLength);
    buff->writeIndex = buff->writeIndex + 1;
    

    if (buff->writeIndex >= buff->buffLength) {
        buff->buffFull = true;
    }
    else {
        buff->buffFull = false;
    }
	systhread_mutex_unlock(x->x_mutex);

}

void readBuff(t_mydspobject* x, ringBuff *buff, float* samples2read)
{
	systhread_mutex_lock(x->x_mutex);
    for (int i = 0; i < buff->nChannels; i++)
    {
        samples2read[i] = buff->buffer[i][buff->readIndex];
    }
    post("current readIndex: %i", buff->readIndex);
    buff->readIndex++;
    // reading the buffer, one sample (all channels) at a time
    while (buff->readIndex >= buff->buffLength)
    {
        buff->readIndex = buff->readIndex - buff->buffLength;
    }
    if (buff->readIndex < 0){
        buff->readIndex = buff->readIndex + buff->buffLength;
    }
	systhread_mutex_unlock(x->x_mutex);
}
void readBuff_with_index(t_mydspobject* x, ringBuff *buff, float* samples2read, int index)
{
	systhread_mutex_lock(x->x_mutex);

	buff->readIndex = index;
	while (buff->readIndex >= buff->buffLength)
    {
        buff->readIndex = buff->readIndex - buff->buffLength;
    }
    if (buff->readIndex < 0){
        buff->readIndex = buff->readIndex + buff->buffLength;
    }
    for (int i = 0; i < buff->nChannels; i++)
    {
        samples2read[i] = buff->buffer[i][buff->readIndex];
    }
    // post("current readIndex in readbuff_with_index: %i", buff->readIndex);
    buff->readIndex++;
    // reading the buffer, one sample (all channels) at a time
	systhread_mutex_unlock(x->x_mutex);
}

void destroyBuff(ringBuff *buff)
{
    for (int i = 0; i < buff->nChannels; i++)
        free(buff->buffer[i]);

    free(buff->buffer);
}

void floatToDoubleBuff(float* floatBuff, double* doubleBuff, int length) {
    // both buffers need to be the same length for this to work
    for (int i = 0; i < length; i++) {
        doubleBuff[i] = (double)floatBuff[i];
    }
}

void ext_main(void* r)
{
	post("ext_main ran");
	// object initialization, note the use of dsp_free for the freemethod, which is required
	// unless you need to free allocated memory, in which case you should call dsp_free from
	// your custom free function.
	t_class* c = class_new("mydsp~", (method)mydsp_new, (method)mydsp_free, (long)sizeof(t_mydspobject), 0L, A_GIMME, 0);

	//class_addmethod(c, (method)mydsp_float, "float", A_FLOAT, 0);
	class_addmethod(c, (method)mydsp_dsp64, "dsp64", A_CANT, 0);
	class_addmethod(c, (method)mydsp_assist, "assist", A_CANT, 0);
	class_addmethod(c, (method)mydsp_free, "free", A_GIMME, 0);
	class_addmethod(c, (method)lsl_inlet_disconnect, "disconnect", A_GIMME, 0);
	class_addmethod(c, (method)lsl_inlet_connect_by_idx, "connect_by_idx", A_DEFFLOAT, NULL); //supposed to end A_NULL
	class_addmethod(c, (method)lsl_inlet_list_all, "list_all", NULL); //supposed to end A_NULL
	class_addmethod(c, (method)lsl_inlet_list_by_property, "list_by_property", A_GIMME, NULL);//supposed to end A_NULL
	class_addmethod(c, (method)lsl_inlet_resolve_by_property, "resolve_by_property", A_GIMME, NULL); //supposed to end A_NULL
	class_addmethod(c, (method)mydsp_perform64, "dsp_add64", 0, NULL);
	class_addmethod(c, (method)mydspthread_cancel, "cancel", 0);
	class_addmethod(c, (method)mydsp_dblclick, "dblclick", A_CANT, 0);


	class_addmethod(c, (method)mydsp_notify, "notify", A_CANT, 0);
	//class_addmethod(c, (method)mainLSL, "lsl", A_CANT, 0);


//DEFINE ATTRIBUTES (idk if here but we will try)
	CLASS_ATTR_SYM(c, "streamname", 0, t_mydspobject, m_stream_name);
	CLASS_ATTR_SYM(c, "streamtype", 0, t_mydspobject, m_stream_type);
	CLASS_ATTR_SYM(c, "streamid", 0, t_mydspobject, m_stream_id);

	class_dspinit(c);
	class_register(CLASS_BOX, c);
	mydsp_class = c;

	ps_buffer_modified = gensym("buffer_modified");

}

void* mydsp_new(t_symbol* s, long argc, t_atom* argv)
{
	post("mydsp_new ran");
	t_mydspobject* x = (t_mydspobject*)object_alloc(mydsp_class);
	t_symbol* buf = 0;

	// in case the thread was already running
	mydspthread_stop(x);
	// defaults
	x->inputs = 1;
	x->outputs = 1;
	attr_args_process(x, argc, argv);

	dsp_setup((t_pxobject*)x, x->inputs);

	 // STUFF FROM LSL INLET PD
	int i, lcl_nout;
	t_symbol* firstarg;

	// defaults
	x->outputs = 66;
	x->connected = 0;

	x->ready = 0;
	x->lag = sys_getblksize();
	x->sr_msp = sys_getsr();
	x->sr_ratio = 1.0;
    x->sr_ratio_inv = 1.0;
	x->lag_lsl = (double)x->lag;
	x->counter = 0;

	bool lslBufferFullFlag = false;

	x->index = 0;


	long dummycounter = argc;
	while (dummycounter > 0) {
		firstarg = atom_getsym(argv);
		// post(argc);

		char* text = NULL;
		long		textsize = 0;
		atom_gettext(argc - 1, argv + 1, &textsize, &text, 0);
		if (!strcmp(firstarg->s_name, "-lag"))
		{

			x->lag = sys_getblksize() * atom_getfloat(argv);
			x->lag_lsl = (double)x->lag;
			argc -= 2;
			argv += 2;
		}
		else if (!strcmp(firstarg->s_name, "-nout"))
		{
			post("nout detected");
			post(firstarg->s_name);
			post(atom_getlong(argv));
			post(atom_getobj(argv));
			//post(atom_getsym(argv));

			lcl_nout = (atom_getlong(argv));
			if ((lcl_nout == 1) || (lcl_nout == 2) || (lcl_nout == 4) ||
				(lcl_nout != 8) || (lcl_nout != 16) || (lcl_nout != 32))
			{

				x->outputs = lcl_nout;

			}
			else
				post("Invalid outlet selection (must be 1, 2, 4, 8, 16, or 32): reverting to 8");
			argc -= 2;
			argv += 2;
		}
		else
		{
			error("%s: unkown flag or argument missing",
				firstarg->s_name);
			//argc--, argv++;
			dummycounter--;
		}
	}
	// setup outlets

	// time stamp outlet
	outlet_new((t_object*)x, "signal");

	x->which = -1;
	x->can_launch_resolver = 1;


	for (i = 0; i < 50; i++)
		x->lsl_info_list[i] = NULL;
	x->lsl_info_list_cnt = 0;

	x->stop_ = 1;
	//post("about to list all");
	lsl_inlet_list_all(x);

	if (x->can_launch_resolver) {
		lsl_inlet_resolve_by_property(x, s, argc, argv);

		//post("about to connect by idx");
		// lsl_inlet_connect_by_idx(x, 0);

		x->outputs = x->nchannels;


		x->lslBuffPtr = &x->LSLbuffer;
		x->outputRingBuffPtr = &x->outputRingBuff;
		x->sizeOfLSLInputBuffer = 2;

		buf = gensym("lslInputBuffer");
		x->w_buf = buffer_ref_new((t_object*)x, buf);


		InitBuff(x->lslBuffPtr, x->outputs, x->sizeOfLSLInputBuffer);
        
	

		// 
		if (x->sr_lsl != 0) {
			x->sizeOfOutputRingBuffer = floor((x->sizeOfLSLInputBuffer) * x->sr_msp / x->sr_lsl);
		}
		else {
			error("LSL stream either not connected or 0 sampling rate, try again");
		}

		InitBuff(x->outputRingBuffPtr, x->outputs, x->sizeOfOutputRingBuffer);


		x->d_internalOutputBuff = calloc(x->outputs, sizeof(double));
		if (!x->d_internalOutputBuff) {                   /* if you allocate -- you must VALIDATE */
			perror("malloc-m");
			return NULL;
		}
		x->f_internalOutputBuff = calloc(x->outputs, sizeof(float));
		if (!x->f_internalOutputBuff) {                   /* if you allocate -- you must VALIDATE */
			perror("malloc-m");
			return NULL;
		}
		x->d_newSampleFramesOutputBuff = calloc(x->outputs, sizeof(float));
		if (!x->d_newSampleFramesOutputBuff) {                   /* if you allocate -- you must VALIDATE */
			perror("malloc-m");
			return NULL;
		}

		x->d_newSampleFramesOutputBuff = malloc(x->outputs * sizeof(double*));
		if (!x->d_newSampleFramesOutputBuff) {                   /* if you allocate -- you must VALIDATE */
			perror("malloc-m");
			return NULL;
		}
		for (int i = 0; i < x->outputs; i++) {
			x->d_newSampleFramesOutputBuff[i] = calloc(64, sizeof(double)); // TODO: ONLY WILL WORK IF SAMPLEFRAMES ACTUALLY == 64
			if (!x->d_newSampleFramesOutputBuff[i]) {                   /* if you allocate -- you must VALIDATE */
				perror("calloc-m");
				return NULL;
			}
		}

		x->curLSLSamps = calloc(x->outputs, sizeof(float));
		x->currentLSLSampleOut = calloc(x->outputs, sizeof(float));
		x->previousLSLSampleOut = calloc(x->outputs, sizeof(float));
		x->nextLSLSampleOut = calloc(x->outputs, sizeof(float));
	}

	for (i = 0; i < x->outputs - 1; i++) {
		post("setting up outlets");
		// channel signal outlets
		outlet_new((t_object*)x, "signal");
	}


	x->x_systhread = NULL;
	systhread_mutex_new(&x->x_mutex, 0);	
	// once we are connected, start the listen thread
		// create new thread + begin execution
	if (x->x_systhread == NULL) {
		post("starting a new thread");
		systhread_create((method)mydspthread_threadproc, x, 0, 0, 0, &x->x_systhread);
	}


	return (x);
}

void mydspthread_stop(t_mydspobject* x)
{
	unsigned int ret;

	if (x->x_systhread) {
		post("stopping our thread");
		x->x_systhread_cancel = true;						// tell the thread to stop
		systhread_join(x->x_systhread, &ret);					// wait for the thread to stop
		x->x_systhread = NULL;
	}
}

void mydspthread_cancel(t_mydspobject* x)
{
	mydspthread_stop(x);									// kill thread if, any
	post("cancelled");
	//outlet_anything(x->outputs, gensym("cancelled"), 0, NULL);
}

void* mydspthread_threadproc(t_mydspobject* x)
{
	// loop until told to stop
	while (x->connected) {
		// test if we're being asked to die, and if so return before we do the work
		if (x->x_systhread_cancel)
			break;

		// if we are connected to an lsl stream, pull sample and add it to our lslBuffer
		if (x->lsl_inlet_obj) {
			x->ts = lsl_pull_sample_f(x->lsl_inlet_obj, x->curLSLSamps, x->outputs, LSL_FOREVER, &(x->ec));
			//post("pulled a sample!!");
            x->index = (x->index + 1) % x->lslBuffPtr->buffLength;
			int curIndex = ((int)x->sr_ratio * (x->index)) % x->outputRingBuffPtr->buffLength;
			post("curindex: %i %i", x->index, curIndex);

			writeBuff(x, x->lslBuffPtr, x->curLSLSamps);
			post("cursamp from lsl: %f", x->curLSLSamps[0]);
			// write to the outputringbuff at curIndex which should be every index * sr_ratio
			writeBuff_with_index(x, x->outputRingBuffPtr, x->curLSLSamps, curIndex);

			if (x->LSLbuffer.buffFull) {
				post("filled the buffer!");
			}
		}
		// qelem_set(x->x_qelem);													// notify main thread using qelem mechanism
	}
	x->x_systhread_cancel = false;							// reset cancel flag for next time, in case
	// the thread is created again
	systhread_exit(0);															// this can return a value to systhread_join();
	return NULL;
}



int curLSLIndex = 2;
bool change = true;
void mydsp_perform64(t_mydspobject* x, t_object* dsp64, double** ins, long numins, double** outs, long numouts, int sampleframes, long flags, void* userparam)
{
	int n;
	if (x->connected == 1)
	{
		// post("cursamp from perform64: %f", x->curLSLSamps[0]);
		// post("current writeindex of lslbuff: %i", x->lslBuffPtr->writeIndex);

		// n is number of samples in block
		n = sampleframes;
		int curStepIdx = 0;
		double d_curStep = 0.0;
		double d_prevVal = 0.0;
		double d_curVal = 0.0;

		// post("curoutputindex: %i", curOutputIndex);
		// check if curoutputindex is within 64 samples to a known sample
		// known samples are every ratio samps at the ratio multiple * a number 0-length of lslbuffptr
		int sampChangeIndex = 0;

		curLSLIndex = x->lslBuffPtr->writeIndex - 1;

		if (change){
			readBuff_with_index(x, x->lslBuffPtr,x->previousLSLSampleOut,curLSLIndex-1);
			readBuff_with_index(x, x->lslBuffPtr,x->currentLSLSampleOut,curLSLIndex);
		}

		if ((curOutputIndex % (int)x->sr_ratio) < 64){
			sampChangeIndex = curOutputIndex % (int)x->sr_ratio;
			// increment curLSLIndex to read next value
			curLSLIndex = curLSLIndex + 1;
			readBuff_with_index(x, x->lslBuffPtr,x->nextLSLSampleOut,curLSLIndex);
			post("within 64 samps of new value! %i", sampChangeIndex);
			change = true;
		}
		else{
			change = false;	
		}
		// if we are within 64 samps, we know which samp in the 64 is gonna be a new number. we need prev, current, and next 
		// if not within 64 samps, just need prev and current to find diff


		for (int i = 0; i < sampleframes; i++) {
			// readBuff(x->outputRingBuffPtr,x->curOutputSamp);
			// post("cursamp2 from perform64: %f", x->curOutputSamp[0]);

			for (int j = 0; j<x->outputs; j++){
				// readBuff(x->outputRingBuffPtr, x->f_internalOutputBuff);
				// floatToDoubleBuff(x->f_internalOutputBuff, x->d_internalOutputBuff, x->outputs);

				// x->d_newSampleFramesOutputBuff[j][i] = x->d_internalOutputBuff[j];
				x->d_newSampleFramesOutputBuff[j][i] =(double)x->currentLSLSampleOut[j];

			}
			// post("curoutputindex: %i", curOutputIndex);
			// x->curOutputIndex = (x->curOutputIndex + 1)%(x->lslBuffPtr->buffLength * (int)x->sr_ratio);
			curOutputIndex = (curOutputIndex + 1)%192000;
        }
        
        for (int i = 0; i < x->outputs; i++) {
            memset(outs[i], 0, 64 * sizeof(double));
            memcpy(outs[i], x->d_newSampleFramesOutputBuff[i], 64 * sizeof(double));
        }
	}
}

void mydsp_assist(t_mydspobject* x, void* b, long m, long a, char* s)
{
	if (m == ASSIST_INLET) { //inlet
		sprintf(s, "I am inlet %ld", a);
	}
	else {	// outlet - i need to add a separate helper for the first outlet which is lsl stream.
		sprintf(s, "Channel %ld", a);
	}
}

// registers a function for the signal chain in Max
void mydsp_dsp64(t_mydspobject* x, t_object* dsp64, t_signal** sp)
{
	//post("mydsp_dsp64 ran");
	object_method(dsp64, gensym("dsp_add64"), x, mydsp_perform64, 0, NULL);
	x->counter = 0;
}

void mydsp_free(t_mydspobject* x)
{
	post("in the free method");
	mydspthread_stop(x);

	destroyBuff(x->lslBuffPtr);
	destroyBuff(x->outputRingBuffPtr);

	destroy_info_list(x);
	lsl_inlet_disconnect(x);
	if (x->lsl_inlet_obj != NULL)lsl_destroy_inlet(x->lsl_inlet_obj);

	dsp_free((t_pxobject*)x);

	free(x->f_internalOutputBuff);
	free(x->d_internalOutputBuff);
	free(x->curLSLSamps);
	free(x->currentLSLSampleOut);
	free(x->nextLSLSampleOut);
	free(x->previousLSLSampleOut);

	object_free(x->w_buf);
}

void lsl_inlet_list_all(t_mydspobject* x) {
	// this used to be a thread.... now a function

	post("lsl_inlet_list_all is running");
	if (x->can_launch_resolver == 0)
		post("LSL outlets cannot be listed at this time. Another process is already at work.");
	else {
		x->can_launch_resolver = 0; // weak locking
		//t_mydspobject* x = (t_mydspobject*)in;
		int listed_count;

		post("Attempting to find LSL outlets on the network...");
		listed_count = lsl_resolve_all(x->lsl_info_list, 50, 5);
		if (listed_count != 0) {
			x->lsl_info_list_cnt = (listed_count > 50 ? 50 : listed_count);
			post_info_list(x);
		}
		else post("no streams available");

		x->can_launch_resolver = 1;
		return;
	}

	//ec = pthread_create(&tid, NULL, list_all_thread, (void *)x);
// if we get here, error
	if (0 == 0) {
		error("Error launching list all thread");
		return;
	}
}

void lsl_inlet_disconnect(t_mydspobject* x) {
	if (x->stop_ != 1) {
		post("disconnecting from %s stream %s (%s)...",
			lsl_get_type(x->lsl_info_list[x->which]),
			lsl_get_name(x->lsl_info_list[x->which]),
			lsl_get_source_id(x->lsl_info_list[x->which]));
		x->stop_ = 1;
		x->which = -1;
		// here we are forced to call the dreaded pthread_cancel
		// because the lsl_inlet waits forever for a new sample to come in
		// if a stream disappears before the thread exits, it will simply
		// stick waiting for a new sample and joining the thread will
		// halt pd in that state
		//pthread_cancel(x->tid);
		// however, we manage all of our resources outside of that thread, so it's cool (I think...)

		if (x->lsl_inlet_obj != NULL) {
			lsl_destroy_inlet(x->lsl_inlet_obj);
			x->lsl_inlet_obj = NULL;
		}
		post("...disconnected");
		x->connected = 0;
		x->ready = 0;
	}
}
//THIS METHOD HAD WEIRD THINGS WITH MSP_ERROR SAYING IT IS AN UNRESOLVED EXTERNAL SYMBOL. SO I REPLACED WITH POST JUST IN CASE.
void lsl_inlet_connect_by_idx(t_mydspobject* x, t_atom_float f)
{

	post("connect by idx ran");
	if (x->lsl_info_list_cnt == 0)
	{
		post("No lsl_info objects available. Please try to resolve available LSL outlets.");
		return;
	}
	else if ((f > x->lsl_info_list_cnt) || (f < 0))
	{
		post("Invalid selection from list of available outlets.");
		return;
	}
	else
	{
		x->which = (int)f;
		post("connecting to %s stream %s (%s)...",
			lsl_get_type(x->lsl_info_list[x->which]),
			lsl_get_name(x->lsl_info_list[x->which]),
			lsl_get_source_id(x->lsl_info_list[x->which]));

		if (lsl_get_channel_format(x->lsl_info_list[x->which]) != cft_double64)
			if (lsl_get_channel_format(x->lsl_info_list[x->which]) != cft_float32)
				if (lsl_get_channel_format(x->lsl_info_list[x->which]) != cft_int32)
				{
					post("requested stream has invalid channel format, only floats, doubles, and 32-bit int data allowed");
					return;
				}
		if (lsl_get_nominal_srate(x->lsl_info_list[x->which]) == 0)
		{
			post("requested stream has invalid nominal sampling rate, this must not be 0");
			return;
		}

		x->lsl_inlet_obj = lsl_create_inlet(x->lsl_info_list[x->which], 300, 1, 1);

		// prepare the ring buffers based on the stream info
		x->nchannels = lsl_get_channel_count(x->lsl_info_list[x->which]);
		//post("%d", x->nchannels);
		if (x->nchannels > x->outputs)
		{
			post("the requested stream has more channels than are available in this msp object");
			return;
		}

		// prepare the upsampling factors based on the stream info
		x->sr_lsl = lsl_get_nominal_srate(x->lsl_info_list[x->which]);
		x->sr_ratio = (double)x->sr_msp / (double)x->sr_lsl;
        x->sr_ratio_inv = (double)x->sr_lsl / (double)x->sr_msp;
		x->lag_lsl = x->sr_ratio_inv * (double)x->lag;
        
		post("LSL Sampling Rate: %d", x->sr_lsl);
        post("LSL lag: %d", x->lag_lsl);
        post("ratio: %f", x->sr_ratio);
		x->connected = 1;
		post("...connected");
	}
}

void destroy_info_list(t_mydspobject* x) {
	int i;
	for (i = 0; i < 50; i++) {
		if (x->lsl_info_list[i] != NULL) {
			lsl_destroy_streaminfo(x->lsl_info_list[i]);
			x->lsl_info_list[i] = NULL;
		}
	}
	x->lsl_info_list_cnt = 0;
}

void post_info_list(t_mydspobject* x) {

	int i;
	int cnt = (x->lsl_info_list_cnt >= 50) ? 50 : x->lsl_info_list_cnt;
	post("----------available lsl streams------------");
	for (i = 0; i < cnt; i++) {
		post("[%d] name: %s  |  type: %s  |  source_id: %s",
			i,
			lsl_get_name(x->lsl_info_list[i]),
			lsl_get_type(x->lsl_info_list[i]),
			lsl_get_source_id(x->lsl_info_list[i]));
	}
}

int prop_resolve(t_mydspobject* x, long argc, t_atom* argv) {
	post("in prop resolve");
	t_symbol* args = atom_getsym(argv);
	int resolved_count = 0;

	post("prop_resolve x: %d, argc %d, argv %d", x, argc, argv);

	for (int i = 0; i < 50; i++) {
		if (x->lsl_info_list[i] == NULL)break;
		else {
			lsl_destroy_streaminfo(x->lsl_info_list[i]);
			x->lsl_info_list[i] = NULL;
		}
	}

	x->lsl_info_list_cnt = 0;

	post(args->s_name);

	char* text = NULL;
	long textsize = 0;
	atom_gettext(argc - 1, argv + 1, &textsize, &text, 0);
	//post(text, textsize);
	if (!strcmp(args->s_name, "-name")) {
		resolved_count = lsl_resolve_byprop(x->lsl_info_list, 50, "name", text, 0, 5);
	}
	else if (!strcmp(args->s_name, "-type"))
		resolved_count = lsl_resolve_byprop(x->lsl_info_list, 50, "type", text, 0, 5);

	else if (!strcmp(args->s_name, "-source_id"))
		resolved_count = lsl_resolve_byprop(x->lsl_info_list, 50, "source_id", text, 0, 5);

	else
		error("lsl_inlet: %s: uniknown flag or argument missing", args->s_name);

	if (resolved_count != 0)
		return(resolved_count);
	else {
		post("could not find any streams of property %s matching value %s", args->s_name, text);
		return 0;
	}
}
void lsl_inlet_list_by_property(t_mydspobject* x, t_symbol* s, long argc, t_atom* argv) {

	int listed_count;
	//post("thread x: %d, argc %d, argv %d", y->x, y->argc, y->argv);
	listed_count = prop_resolve(x, argc, argv);

	if (listed_count != 0) {
		x->lsl_info_list_cnt = (listed_count > 50 ? 50 : listed_count);
		post_info_list(x);
	}
}
void lsl_inlet_resolve_by_property(t_mydspobject* x, t_symbol* s, long argc, t_atom* argv) {

	x->lsl_info_list_cnt = 0;
	x->lsl_info_list_cnt = prop_resolve(x, argc, argv);
	if (x->lsl_info_list_cnt != 0)
		lsl_inlet_connect_by_idx(x, 0);
}

t_max_err mydsp_notify(t_mydspobject* x, t_symbol* s, t_symbol* msg, void* sender, void* data)
{
	t_symbol* attrname;
	if (msg == gensym("attr_modified")) {       // check notification type
		attrname = (t_symbol*)object_method((t_object*)data, gensym("getname"));      // ask attribute object for name
		object_post((t_object*)x, "changed attr name is %s", attrname->s_name);

	}

	if (msg == ps_buffer_modified)
		x->w_buffer_modified = true;
	return buffer_ref_notify(x->w_buf, s, msg, sender, data);

	return 0;
}

void mydsp_dblclick(t_mydspobject* x)
{
	buffer_view(buffer_ref_getobject(x->w_buf));
}

