#include <vector>
#include <libfreenect/libfreenect.h>
#include <libfreenect/libfreenect_audio.h>
#include <stdio.h>
#include <signal.h>
#include <stdexcept>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <netdb.h>
#include <iostream>
#include <regex.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string>
#include <stdlib.h>
#include <locale.h>
#include <signal.h>
#include <sys/wait.h>
#include <fstream>

using namespace std;

#define UNUSED(x) (void)(x)
#define DEBUG(...) printf(__VA_ARGS__);fflush(stdout)
#define INFO(...) printf(__VA_ARGS__);fflush(stdout)

volatile sig_atomic_t continue_loop = 1;
FILE *f;
freenect_context* f_ctx;
freenect_device* f_dev = nullptr;
int child_socket = 0;

void in_callback(freenect_device* dev, int num_samples,
                 int32_t* mic1, int32_t* mic2,
                 int32_t* mic3, int32_t* mic4,
                 int16_t* cancelled, void *unknown);
void sighandler(int signo);
int start_server(int port);
void open_kinect ();
int read ();
int send_test ();

int main ()
{
     open_kinect ();
     start_server (1235);
     return 0;
}

int start_server(int port)
{
    struct sockaddr_in income_sock;
    income_sock.sin_addr.s_addr = INADDR_ANY;
    income_sock.sin_port = htons(port);
    income_sock.sin_family = PF_INET;
    int in_socket = 0;

    if((in_socket = socket(PF_INET, SOCK_STREAM, 0)) < 0)
    {
        fprintf(stderr,"Unable to create socket\n");
        return 1;
    }

    if(bind(in_socket, (struct sockaddr *)&income_sock, sizeof(income_sock)) < 0 )
    {
        fprintf(stderr, "Unable to bind socket and IP address\n");
        return 2;
    }

    if((listen(in_socket, 50)) != 0)
    {
        fprintf(stderr, "Unable to bind socket and IP address\n");
        return 3;
    }

    socklen_t income_sock_size;
    income_sock_size = sizeof(income_sock);
    while (continue_loop)
    {
        INFO ("waiting for new connection\n");
        if((child_socket = accept(in_socket, (struct sockaddr *)&income_sock, &income_sock_size)) < 0) {
            return 3;
        }
        INFO ("starting sending data\n");
        read ();
        //send_test ();
        INFO ("disconnected\n");


    }

    if(close(in_socket) < 0)
    {
        fprintf(stderr, "Unable to fork process\n");
        return 4;
    }

    return 0;
}

int send_test ()
{
     ifstream file;
	file.open ("tmp.raw", ios_base::binary | ios_base::in);
     while (!file.eof()) {
          char buf_file[512];
		file.read (buf_file, 512);
		if (write (child_socket, buf_file, 512) < 0) {
               free (buf_file);
               file.close ();
               fprintf(stderr, "Communication with client failed\n");
               return 1;
          }
          //sleep (1);
     }
     return 0;
}

void open_kinect ()
{
     if (freenect_init(&f_ctx, NULL) < 0) {
          throw runtime_error ("freenect_init() failed");
     }
     //freenect_set_log_level(f_ctx, FREENECT_LOG_SPEW);
     freenect_select_subdevices(f_ctx, FREENECT_DEVICE_AUDIO);

     int nr_devices = freenect_num_devices (f_ctx);
     printf ("Number of devices found: %d\n", nr_devices);
     if (nr_devices < 1) {
          freenect_shutdown(f_ctx);
          throw runtime_error ("no devices found");
     }

     int user_device_number = 0;
     if (freenect_open_device(f_ctx, &f_dev, user_device_number) < 0) {
          freenect_shutdown(f_ctx);
          throw runtime_error ("could not open device");
     }
     freenect_set_audio_in_callback(f_dev, in_callback);
     signal (SIGINT, sighandler);
     f = fopen ("tmp.raw", "wb");
}

void shutdown_kinect ()
{
     freenect_shutdown(f_ctx);
     fclose (f);
}

void in_callback(freenect_device* dev, int num_samples,
                 int32_t* mic1, int32_t* mic2,
                 int32_t* mic3, int32_t* mic4,
                 int16_t* cancelled, void *unknown) {
     UNUSED(dev);
     UNUSED(unknown);
     UNUSED(mic1);
     UNUSED(mic2);
     UNUSED(mic3);
     UNUSED(mic4);
     vector<int32_t> vec_32;
     vec_32.assign (mic2, mic2 + num_samples);
     vector<int16_t> vec_16;
     for (unsigned i = 0; i < vec_32.size (); ++i) {
          vec_16.push_back ((vec_32[i] >> 16) * 16);
     }
     fwrite (&vec_16[0], sizeof (int16_t), num_samples, f);
     //callable (&vec_16[0], num_samples);
     write (child_socket, &vec_16[0], num_samples * sizeof (int16_t));
     INFO ("sent %d frames\n", num_samples);
}

int read ()
{
     freenect_start_audio(f_dev);
     while (continue_loop && freenect_process_events(f_ctx) >= 0) {
     }
     return 0;
}

void sighandler(int signo)
{
     continue_loop = signo = 0;
     INFO ("interrupted by user, leaving\n");
     sleep (2);
     exit (0);
}
