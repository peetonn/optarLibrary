//
// test-optar.cpp
//
//  Created by Peter Gusev on 12 November 2019.
//  Copyright 2013-2019 Regents of the University of California
//

#include "optar.hpp"

#include <iostream>
#include <sstream>
#include <ctype.h>
#include <chrono>
#include <execinfo.h>
#include <vector>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#if HAVE_DOCOPT
#include <docopt/docopt.h>

static const char USAGE[] =
R"(test-optar.

    An app for testing and developing optar library.

    Usage:
      test-optar <rgba_file> --size=<frame_size> [ --fps=<fps> ] [ --i ] [ --preview ]

    Arguments:
      <rgba_file>               Raw video (RGBA) file.
    Options:
      -s --size=<frame_size>    Frame size <width>x<height>
      --fps=<fps>               Frame rate to run video [default: 60]
      --i                       Interactive mode (frames read on key press)
      --preview                 Shows debug preview images (needs to be
                                    compiled with DEBUG=1 flag)
)";

#endif

using namespace std;
using namespace optar;

bool mustExit = false;
void runTest(const string &videoFile, uint16_t w, uint16_t h, double fps,
             bool interactive, bool debugPreview);
int getSize(const string& sizeStr, uint16_t& w, uint16_t& h);
size_t getFrameSize(int w, int h);
int readFrame(FILE* f, uint8_t* buf, uint16_t w, uint16_t h);
void printStats(int frameIdx, const map<string, double>& stats);

void handler(int sig)
{
    void *array[10];
    size_t size;

    if (sig == SIGABRT || sig == SIGSEGV)
    {
        fprintf(stderr, "Received signal %d:\n", sig);
        // get void*'s for all entries on the stack
        size = backtrace(array, 10);
        // print out all the frames to stderr
        backtrace_symbols_fd(array, size, STDERR_FILENO);
        exit(1);
    }
    else
        mustExit = true;
}

int main(int argc, char **argv)
{
    signal(SIGABRT, handler);
    signal(SIGSEGV, handler);
    signal(SIGINT, &handler);
    signal(SIGUSR1, &handler);

    cout << "testing optar library version " << getLibraryVersion() << endl;

#if HAVE_DOCOPT

    std::map<std::string, docopt::value> args
            = docopt::docopt(USAGE,
                             { argv + 1, argv + argc },
                             true,               // show help if requested
                             (string("test-optar ")+string(PACKAGE_VERSION)).c_str());  // version string

    for(auto const& arg : args)
        std::cout << arg.first << " " <<  arg.second << std::endl;
    
    uint16_t w,h;
    double fps = (double)atof(args["--fps"].asString().c_str());
    
    if (getSize(args["--size"].asString(), w, h))
    {
        runTest(args["<rgba_file>"].asString(), w, h, fps, args["--i"].asBool(), args["--preview"].asBool());
    }

#else
    cout << "test-optar requires docopt dependency. exiting now." << endl;
    return 1;
#endif

    return 0;
}

void runTest(const string &videoFile, uint16_t w, uint16_t h, double fps,
             bool interactive, bool debugPreview)
{
    FILE *f = fopen(videoFile.c_str(), "rb");
    if (!f)
    {
        cout << "failed to open video " << videoFile << strerror(errno) << endl;
        return;
    }
 
    cout << "reading " << videoFile << " res " << w << "x" << h << " at " << fps << " FPS" << endl;
    
    uint8_t *rgbaData = (uint8_t*) malloc(getFrameSize(w,h));
    int iterUs = int(1E6 / fps); // microsecond per iteration
    auto t = chrono::high_resolution_clock::now();
    int frameIdx = 0;
    
    
    OptarClient::Settings s;
    s.orbLevelsNumber_ = 10;
    s.showDebugImage_ = interactive || debugPreview;
    OptarClient optarClient(s);
    
    while (!mustExit)
    {
        if (readFrame(f, rgbaData, w, h))
        {
            frameIdx++;
            optarClient.processTexture(w, h, rgbaData);
            printStats(frameIdx, optarClient.getStats());
        }
        else
            break;
        
        auto end = chrono::high_resolution_clock::now();
        double procUs = chrono::duration_cast<chrono::microseconds>(end-t).count();
        
        if (interactive)
        {
            cout << " Press [Enter] to read next frame..." << flush;
            getchar();
        }
        else
        {
            if (procUs < iterUs)
                usleep(iterUs - procUs);
//        else
//            cout << "can't keep up with " << fps << " FPS: last iteration took " << procUs / 1000 << "ms" << endl;
        }
        
        t = end;
    }
    
    if (rgbaData) free(rgbaData);
}

int readFrame(FILE* f, uint8_t* buf, uint16_t w, uint16_t h)
{
    size_t frameSize = getFrameSize(w,h);
    int nIter = 0;
    do {
        ++nIter;
        
        size_t c = fread(buf, 1, frameSize, f);
        
        if (c == 0 || c < frameSize)
            return 0;
        frameSize -= c;
    } while (frameSize != 0);
    
    return 1;
}

size_t getFrameSize(int w, int h)
{
    return w*h*4;
}

int getSize(const string& sizeStr, uint16_t& w, uint16_t& h)
{
    istringstream ss(sizeStr);
    string k;
    int idx = 0;

    while(getline(ss, k, 'x') && idx < 2)
    {
        int val;

        try {
            val = stoi(k);
            if (val > 0)
            {
                if (idx == 0)
                    w = val;
                else
                    h = val;
                idx++;
            }
        }
        catch(std::exception&)
        {
            break;
        }
    }

    return (idx == 2);
}

void printStats(int frameIdx, const map<string, double>& stats)
{
    cout << "\r"
    << "[ "
    << "frame " << setw(6) << frameIdx << " "
    << fixed
    << setw(6) << setprecision(2) << stats.at("optarProc") << "us"
    << setw(6) << " kp " << stats.at("orbKp")
    << " ]"
    << flush;
    
}
