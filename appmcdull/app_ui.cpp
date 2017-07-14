// app_ui.cpp

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define UI_FL

int main_app(int argc, char* argv[])
;

typedef struct {
	char filename[80];
}
param_t;

param_t _param;

char* parse(const char* filename, char* name, int& l1, int& l2)
{
	sprintf(name, filename);
	char* p = strchr(name, '_');
	sscanf(p, "_%dx%d", &l1, &l2);
	*p = '\0';

	return name;
}

void app(param_t* param)
{
	const char* filename = param->filename;

	char name[80];
	int l1, l2;

	parse(filename, name, l1, l2);
	int argc=0;
	char _argv[16][80];
	char* argv[16];

	sprintf(_argv[argc++], "%s", "app.exe");
	sprintf(_argv[argc++], "%s", name);
	sprintf(_argv[argc++], "%d", l1);
	sprintf(_argv[argc++], "%d", l2);

	for (int k=0; k<argc; k++) argv[k] = _argv[k];

	main_app(argc, argv);
}

int run()
{
	sprintf(_param.filename, "../../../work/data/paris_352x288.yuv");
	app(&_param);

	return 0;
}

#if defined( UI_FL )

#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Menu_Bar.H>
#include <FL/Fl_File_Chooser.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_Input_Choice.H>

void Fl_callback_open(Fl_Widget*, void*) {
    Fl_File_Chooser chooser(
		"../../../work/data",		// directory
		"*.yuv",					// filter
		Fl_File_Chooser::SINGLE,	// chooser type
		"application"				// title
		);
    chooser.show();

    while(chooser.shown()) { Fl::wait(); }

    if ( chooser.value() == NULL ) { return; }

	sprintf(_param.filename, chooser.value());
}

void Fl_callback_exit(Fl_Widget*, void*) {
    exit(0);
}

void Fl_callback_run(Fl_Widget*, void*) {
	printf("filename = %s\n", _param.filename);

	app(&_param);
}



int Fl_run()
{
	Fl_Window win(300, 180, "Fl_File_Chooser");
	Fl_Menu_Bar menubar(0,0,300,25);
	menubar.add("open", 0, Fl_callback_open);
	menubar.add("run ", 0, Fl_callback_run);
	menubar.add("exit", 0, Fl_callback_exit);

	win.show();
	return(Fl::run());
}

#endif // UI_FL

int main() {
#if defined(UI_FL)
	Fl_run();
#else  // UI_FL
	run();
#endif // UI_FL
}
