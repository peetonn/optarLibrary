//
// test-optar.cpp
//
//  Created by Peter Gusev on 12 November 2019.
//  Copyright 2013-2019 Regents of the University of California
//

#include "optar.hpp"
#include <iostream>
#include <docopt/docopt.h>


using namespace std;
using namespace optar;

int main(int argv, char **argc)
{
    cout << "testing optar library version " << getLibraryVersion() << endl;

    OptarClient client;

    return 0;
}
