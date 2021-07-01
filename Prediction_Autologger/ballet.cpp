#include "ballet_includes.h"

int main(int argc, char **argv)
{
    ballet::Mercator origin;
    ballet::Balloon balloon;
    double epoch;

    try
    {
        origin.lat = std::stod(argv[1]);
        origin.lon = std::stod(argv[2]);
        origin.alt = std::stod(argv[3]);
        balloon.setAscentRate(std::stod(argv[4]));
        epoch = std::stod(argv[5]);
        balloon.setBurstAltitude(34000);
        balloon.setOrigin(origin);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return -1;
    }
    std::cout << "Check 1"<<std::endl;
    ballet::Environment environment("home/tony/HAB/Prediction_Autologger/");
    std::cout << "Check 2"<<std::endl;
    ballet::KML kml("../", std::to_string(int(epoch)));

    while (environment.inEnvironment(balloon.getCurrentMercator(), epoch))
    {
        ballet::Node local_node = environment.subsampleVectorFieldForNode(balloon.getCurrentMercator(), epoch);
        balloon.propagateForward(local_node, epoch);
        kml.outputFile() << std::setprecision(10) << balloon.getCurrentMercator().lon << "," << balloon.getCurrentMercator().lat << "," << balloon.getCurrentMercator().alt << "\n";
    }

    std::cout << "Done!" << std::endl;
    kml.closeKML();
    
    return 0;
}
