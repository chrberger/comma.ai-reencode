/*
 * Copyright (C) 2018  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include "comma.ai.hpp"

#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

int32_t main(int32_t argc, char **argv) {
    int32_t retCode{0};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ( (0 == commandlineArguments.count("in")) || (0 == commandlineArguments.count("out")) ) {
        std::cerr << argv[0] << " reencodes an existing comma.ai dataset into the OpenDLV Standard Message Set." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --in=<existing recording> --out=<output> [--verbose]" << std::endl;
        std::cerr << "Example: " << argv[0] << " --in=myRec.rec --out=myNewRec.rec" << std::endl;
        retCode = 1;
    } else {
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};

        (void)VERBOSE;
        std::string recFile{commandlineArguments["in"]};

        std::fstream fin(recFile, std::ios::in|std::ios::binary);
        if (fin.good()) {
            while (fin.good()) {
                auto retVal{cluon::extractEnvelope(fin)};
                if (retVal.first) {
                    if (static_cast<int32_t>(commaai::Vehicle::ID()) == retVal.second.dataType()) {
                    }
                }
            }
        }
        else {
            std::cerr << "[" << argv[0] << "] '" << recFile << "' could not be opened." << std::endl;
        }
    }
    return retCode;
}
