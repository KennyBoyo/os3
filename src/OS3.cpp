#include <ctime>
#include <chrono>
#include "OS3.h"

using namespace SimTK;
using namespace OpenSim;
using namespace std;
using namespace OS3;

typedef chrono::high_resolution_clock Clock;


int main(int argc, char** argv) {
    cout << "OS3" << endl;
    cout << get_current_dir_name() << endl;

    OS3Engine engine;

    engine.init();

    engine.set_state(ENGINE_RUN);

    while (engine.get_state()) {
        switch(engine.get_state()) {
            case(ENGINE_RUN):
                cout << "Engine Running" << endl;
                engine.loop();
                break;

            case(ENGINE_STANDBY):
                break;
            
            case(ENGINE_END_EXPERIMENT):
                break;
        }
    }
    return true;
}

