#include <tiny-3D-loader.hpp>

int main(int argc, char** argv)  {

  std::vector<tiny3Dloader::scene::Scene*> scenes;
  tiny3Dloader::Importer importer;
  bool success = importer.load("models/BoomBox.gltf", "models/", scenes);

  if (!success)
    std::cerr << importer.getError() << std::endl;
  else
    std::cout << "Loading Lantern.gltf file success..." << std::endl;

  return 0;

}
