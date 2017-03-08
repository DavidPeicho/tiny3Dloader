#include <tiny-3D-loader.hpp>

int main(int argc, char** argv)  {

  std::vector<tiny3Dloader::scene::Node*> roots;
  tiny3Dloader::Importer importer;
  importer.load("models/Lantern.gltf", "models/", roots);

  return 0;

}
