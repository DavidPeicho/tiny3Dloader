#include <tiny-3D-loader.hpp>

int main(int argc, char** argv)  {

  /*std::vector<tiny3Dloader::scene::Node*> roots;
  tiny3Dloader::Importer importer;
  bool success = importer.load("models/Lantern.gltf", "models/", roots);

  if (!success)
    std::cerr << importer.getError() << std::endl;
  else
    std::cout << "Loading Lantern.gltf file success..." << std::endl;*/

  std::vector<tiny3Dloader::scene::Node*> roots;
  tiny3Dloader::Importer importer;
  importer.load("models/MissingBin.gltf", "models/", roots);

  return 0;

}
