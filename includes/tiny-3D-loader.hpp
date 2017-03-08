/*
 * Copyright (c) 2017 David Peicho

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once

#include <json.hpp>

#include <iostream>
#include <fstream>
#include <string>

#include <unordered_map>

namespace tiny3Dloader {

  namespace scene {

    struct Mesh {
      std::string         name;
      std::vector<float>  vertices;
      std::vector<float>  normals;
      std::vector<float>  texcoords;
      std::vector<uint>   indices;
    };

    struct Node {

      ~Node();

      std::string         name;
      std::vector<Mesh*>  meshes;
      std::vector<Node*>  children;

      std::vector<float>  translation;
      std::vector<float>  rotation;
      std::vector<float>  scale;
      std::vector<float>  matrix;
    };

    Node::~Node() {

      for (const auto& meshPtr : meshes) {
        delete meshPtr;
      }

      meshes.clear();
      children.clear();

    }

  } // namespace scene

  class Loader {

    public:
      virtual std::vector<scene::Node*>
      load(std::string pathToFile, std::string assetsFolderPath = "") = 0;

      void
      freeScene();

      inline void
      setDebug(bool debug) { debug_ = debug; }

    public:
      inline const std::string&
      getError() const { return errorStr_; }

    protected:
      inline void
      debug(const std::string& message) {

        if (debug_)
          std::cout << message << std::endl;

      };

      inline void
      logError(const std::string& errorMsg) { errorStr_ += errorMsg + "\n"; }

    protected:
      std::string                             assetsFolderPath_;
      std::string                             errorStr_;

      std::unordered_map<uint, scene::Node*>  nodesMap_;

    private:
      bool                                    debug_;
  };

  class glTFLoader : public Loader {

    enum ComponentType {
      BYTE = 5120,
      UBYTE = 5121,
      SHORT = 5122,
      USHORT = 5123,
      FLOAT = 5126
    };

    enum Type {
      SCALAR = 1,
      VEC2 = 2,
      VEC3 = 3,
      VEC4 = 4,
      MAT2 = 4,
      MAT3 = 9,
      MAT4 = 16,
    };

    public:
      std::vector<scene::Node*>
      load(std::string pathToFile, std::string assetsFolderPath = "") override;

    private:
      void
      processNode(uint nodeId);

      void
      processTransform(uint nodeId);

      void
      processMesh(uint nodeId, uint meshId);

      template <typename T>
      void
      processAccessor(uint accessorId, std::vector<T>& result);

      bool
      registerBuffer(uint bufferId);

    private:
      nlohmann::json                          json_;
      std::unordered_map<uint, uint8_t*>      binaryFiles_;

  };

  class Importer {

    public:
      bool
      load(std::string pathToFile, std::vector<scene::Node*>& roots);

      bool
      load(std::string pathToFile, std::string assetsFolderPath, std::vector<scene::Node*>& roots);

      void
      freeScene();

    private:
      Loader* loader_;

  };

  bool
  Importer::load(std::string pathToFile, std::vector<scene::Node *> &roots) {

    return load(pathToFile, "", roots);

  }

  bool
  Importer::load(std::string pathToFile, std::string assetsFolderPath, std::vector<scene::Node *> &roots) {

    std::string ext = pathToFile.substr(pathToFile.find_last_of(".") + 1);
    if (ext == "gltf")
      this->loader_ = new glTFLoader;
    else
      return false;

    this->loader_->load(pathToFile, assetsFolderPath);
    return this->loader_->getError().empty();
  }

  void
  Loader::freeScene() {

    for (const auto& nodePtr : nodesMap_) {
      delete nodePtr.second;
    }

  }

  std::vector<scene::Node*>
  glTFLoader::load(std::string pathToFile, std::string assetsFolderPath) {

    std::ifstream stream(pathToFile);
    stream >> this->json_;

    this->assetsFolderPath_ = assetsFolderPath;

    // TODO: Check if every sections are created

    uint nodeId = 0;
    auto& sceneNodes = this->json_["nodes"];

    // Allocates every nodes and add
    // them to the unordered map.
    for (const auto& jsonNode : sceneNodes) {
      scene::Node* node = new scene::Node;
      this->nodesMap_[nodeId] = node;

      ++nodeId;
    }

    nodeId = 0;
    for (const auto& jsonNode : sceneNodes) {
        processNode(nodeId);
        ++nodeId;
    }

    std::vector<scene::Node*> roots;

    auto& scenes = this->json_["scenes"];
    if (scenes.count("nodes")) {
      for (uint nodeId : scenes["nodes"]) {
        roots.push_back(nodesMap_[nodeId]);
      }
    }

    return roots;
  }

  void
  glTFLoader::processNode(uint nodeId) {

    auto& jsonNode = this->json_["nodes"][nodeId];
    auto& node = nodesMap_[nodeId];
    node->name = (jsonNode.count("name")) ? jsonNode["name"] : "";

    debug("Node: " + std::to_string(nodeId) + " [" + node->name + "]");

    if (jsonNode.find("name") != jsonNode.end()) {
      node->name = jsonNode["name"];
    }

    if (jsonNode.find("mesh") != jsonNode.end()) {
      processMesh(nodeId, jsonNode["mesh"]);
    }

    processTransform(nodeId);

    // Handles parenting by linking
    // the children to the current node.
    if (jsonNode.find("children") != jsonNode.end()) {
      for (const auto childId : jsonNode["children"]) {
        const auto& it = this->nodesMap_.find(childId);
        if (this->nodesMap_.find(childId) != this->nodesMap_.end()) {
          node->children.push_back(it->second);
        }
      }
    }
  }

  void
  glTFLoader::processTransform(uint nodeId) {

    auto& parentNode = nodesMap_[nodeId];
    auto& jsonNode = this->json_["nodes"][nodeId];

    if (jsonNode.count("translation")) {
      parentNode->translation = jsonNode["translation"].get<std::vector<float>>();
    }
    if (jsonNode.count("rotation")) {
      parentNode->rotation = jsonNode["rotation"].get<std::vector<float>>();
    }
    if (jsonNode.count("scale")) {
      parentNode->scale = jsonNode["scale"].get<std::vector<float>>();
    }
    if (jsonNode.count("matrix")) {
      parentNode->matrix = jsonNode["matrix"].get<std::vector<float>>();
    }
  }

  void
  glTFLoader::processMesh(uint nodeId, uint meshId) {

    // TODO: Remove duplicate meshes allocated severeal times
    auto& parentNode = nodesMap_[nodeId];

    auto& jsonMesh = this->json_["meshes"][meshId];
    auto& jsonPrimitives = jsonMesh["primitives"];

    this->debug("\tMesh: " + std::to_string(meshId));

    for (const auto& jsonPrimitive : jsonPrimitives) {
      scene::Mesh* mesh = new scene::Mesh;
      const auto &attributes = jsonPrimitive["attributes"];

      uint normalId = attributes["NORMAL"].get<uint>();
      uint positionId = attributes["POSITION"].get<uint>();

      processAccessor(normalId, mesh->normals);
      processAccessor(positionId, mesh->vertices);

      // Processes all texcoords
      for (uint i = 0; attributes.find("TEXCOORD_" + i) != attributes.end(); ++i) {
        uint texcoordId = attributes["TEXCOORD_" + i].get<uint>();
        processAccessor(texcoordId, mesh->texcoords);
      }

      // Processes primitive indexes
      uint indicesId = jsonPrimitive["indices"].get<uint>();
      processAccessor(indicesId, mesh->indices);

      parentNode->meshes.push_back(mesh);
    }
  }

  template <typename T>
  void
  glTFLoader::processAccessor(uint accessorId,  std::vector<T>& result) {

    static std::map<std::string, glTFLoader::Type> typeTable = {
      { "SCALAR", glTFLoader::Type::SCALAR },
      { "VEC2", glTFLoader::Type::VEC2 },
      { "VEC3", glTFLoader::Type::VEC3 },
      { "VEC4", glTFLoader::Type::VEC4 },
      { "MAT2", glTFLoader::Type::MAT2 },
      { "MAT3", glTFLoader::Type::MAT3 },
      { "MAT4", glTFLoader::Type::MAT4 },
    };

    static std::map<uint, uint> typeToSize = {
      { ComponentType::BYTE, 1 },
      { ComponentType::UBYTE, 1 },
      { ComponentType::SHORT, 2 },
      { ComponentType::USHORT, 2 },
      { ComponentType::FLOAT, 4 }
    };

    const auto& jsonAccessors = this->json_["accessors"];
    const auto& jsonBufferViews = this->json_["bufferViews"];
    const auto& accessor = jsonAccessors[accessorId];
    const auto& bufferView = jsonBufferViews[accessor["bufferView"].get<uint>()];

    uint count = accessor["count"];
    uint componentType = accessor["componentType"].get<uint>();
    uint bytePerComponent = typeToSize[componentType];
    uint numComponents = typeTable[accessor["type"]];
    uint elementSize = numComponents * bytePerComponent;
    uint totalSize = elementSize * count;
    uint byteStride = (accessor.count("byteStride")) ? accessor["byteStride"].get<uint>() : elementSize;

    uint bufferId = bufferView["buffer"].get<uint>();
    uint offset = accessor["byteOffset"].get<uint>() + bufferView["byteOffset"].get<uint>();

    if (this->binaryFiles_.find(bufferId) == this->binaryFiles_.end()) {

      if (!registerBuffer(bufferId)) return;

    }

    uint8_t * rawData = this->binaryFiles_[bufferId];
    if (rawData == nullptr)
      return;

    if (sizeof(T) < bytePerComponent)
      return;

    result.reserve(count);
    if (byteStride == elementSize && sizeof(T) == bytePerComponent) {
      copy(&rawData[0], &rawData[count], back_inserter(result));
    }
    /*else {
      for (size_t i = 0; i < count; ++i) {
        memcpy(rawData + i, rawData + i * byteStride, elementSize);
        // result[i * byteStride]
      }
    }*/

    /*for (uint t = 0; t < result.size(); t++) {
      std::cout << result[t] << ' ';
      if (t % 3 == 0)
        std::cout << std::endl;
    }
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << std::endl;*/

    //std::string debugStr = "\t-> Accessing buffer " + bufferId;
    /*debugStr += + "`: " + count;
    debugStr += "|" + bytePerComponent;*/
    this->debug("\t-> Accessing buffer '" + std::to_string(bufferId) + "' > " + std::to_string(count));
  }

  bool
  glTFLoader::registerBuffer(uint bufferId) {

    const auto& jsonBuffers = this->json_["buffers"];
    const auto& jsonBuffer = jsonBuffers[bufferId];

    std::string uri = this->assetsFolderPath_ + jsonBuffer["uri"].get<std::string>();
    std::ifstream ifs(uri, std::ios::in
                      | std::ios::binary
                      | std::ios::ate);

    // TODO: Handle binary file not read
    if (ifs.fail()) {
      this->logError("Invalid File: '" + uri + " not found.");
      return false;
    }

    std::ifstream::pos_type fileSize = ifs.tellg();
    char* buffer = new char[fileSize];

    ifs.seekg(0, std::ios::beg);
    // TODO: Handle read fail
    ifs.read(buffer, fileSize);

    this->binaryFiles_[bufferId] = reinterpret_cast<uint8_t*>(buffer);
    return true;
  }

} // namespace tiny3Dloader

