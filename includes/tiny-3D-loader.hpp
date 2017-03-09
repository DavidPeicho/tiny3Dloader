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

    struct Scene {
      std::string         name;
      std::vector<Node*>  nodes;
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
      virtual void
      load(std::string pathToFile, std::vector<scene::Scene*> scenes);

      virtual void
      load(const std::string& pathToFile,
           const std::string& assetsFolderPath,
           std::vector<scene::Scene*>& scenes) = 0;

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
      logError(const std::string& errorMsg) {
        errorStr_ += "Error: ";
        errorStr_ += errorMsg + "\n";
      }

      inline void
      logMissingFile(const std::string& file) {
        logError("Invalid File: '" + file + " not found.");
      }

    protected:
      std::string                             assetsFolderPath_;
      std::string                             errorStr_;

      std::unordered_map<uint, scene::Node*>  nodesMap_;

    private:
      bool                                    debug_ = false;
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

    enum Target {
      ARRAY_BUFFER = 34962,
      ELEMENT_ARRAY_BUFFER = 34963,
    };

    public:
      void
      load(const std::string& pathToFile,
           const std::string& assetsFolderPath,
           std::vector<scene::Scene*>& scenes) override;

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

      bool
      checkValidity();

      bool
      checkMissingKey(const std::string& key);

    private:
      nlohmann::json                          json_;
      std::unordered_map<uint, uint8_t*>      binaryFiles_;

  };

  class Importer {

    public:
      bool
      load(const std::string& pathToFile, std::vector<scene::Scene*>& scenes);

      bool
      load(const std::string& pathToFile, const std::string& assetsFolderPath,
           std::vector<scene::Scene*>& scenes);

      void
      freeScene();

    public:
      inline const std::string
      getError() const {

        if (loader_) return loader_->getError();
        return "";

      }

    private:
      Loader* loader_;

  };

  bool
  Importer::load(const std::string& pathToFile,
                 std::vector<scene::Scene*>& scenes) {

    return load(pathToFile, "", scenes);

  }

  bool
  Importer::load(const std::string& pathToFile,
                 const std::string& assetsFolderPath,
                 std::vector<scene::Scene*>& scenes) {

    std::string ext = pathToFile.substr(pathToFile.find_last_of(".") + 1);
    if (ext == "gltf") {
      this->loader_ = new glTFLoader;
    } else {
      return false;
    }

    this->loader_->load(pathToFile, assetsFolderPath, scenes);
    return this->loader_->getError().empty();
  }

  void
  Loader::load(std::string pathToFile, std::vector<scene::Scene *> scenes) {

    this->load(pathToFile, "", scenes);

  }

  void
  Loader::freeScene() {

    for (const auto& nodePtr : nodesMap_) {
      delete nodePtr.second;
    }

  }

  void
  glTFLoader::load(const std::string& pathToFile,
                   const std::string& assetsFolderPath,
                   std::vector<scene::Scene*>& scenesResult) {

    std::ifstream stream(pathToFile);
    if (stream.fail()) {
      this->logMissingFile(pathToFile);
      return;
    }
    stream >> this->json_;

    this->assetsFolderPath_ = assetsFolderPath;

    if (!checkValidity()) return;

    uint nodeId = 0;
    auto& jsonNodes = this->json_["nodes"];

    // Allocates every nodes and add
    // them to the unordered map.
    for (const auto& jsonNode : jsonNodes) {
      scene::Node* node = new scene::Node;
      this->nodesMap_[nodeId] = node;

      ++nodeId;
    }

    nodeId = 0;
    for (const auto& jsonNode : jsonNodes) {
        processNode(nodeId);
        ++nodeId;
    }

    auto& scenes = this->json_["scenes"];
    for (const auto& jsonScene : scenes) {
      if (jsonScene.count("nodes")) {
        scene::Scene* scene = new scene::Scene;
        if (jsonScene.count("name")) scene->name = jsonScene["name"];

        const auto& nodes = jsonScene["nodes"];
        for (uint nodeId : nodes) {
          scene->nodes.push_back(nodesMap_[nodeId]);
        }

        scenesResult.push_back(scene);
      }
    }

  }

  void
  glTFLoader::processNode(uint nodeId) {

    auto& jsonNode = this->json_["nodes"][nodeId];
    auto& node = nodesMap_[nodeId];
    node->name = (jsonNode.count("name")) ? jsonNode["name"] : "";

    debug("Node: " + std::to_string(nodeId) + " [" + node->name + "]");

    if (jsonNode.count("name")) {
      node->name = jsonNode["name"];
    }

    if (jsonNode.count("mesh")) {
      processMesh(nodeId, jsonNode["mesh"]);
    }

    processTransform(nodeId);

    // Handles parenting by linking
    // the children to the current node.
    if (jsonNode.count("children")) {
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

    auto& parent = nodesMap_[nodeId];
    auto& jsonNode = this->json_["nodes"][nodeId];

    if (jsonNode.count("translation")) {
      parent->translation = jsonNode["translation"].get<std::vector<float>>();
    }
    if (jsonNode.count("rotation")) {
      parent->rotation = jsonNode["rotation"].get<std::vector<float>>();
    }
    if (jsonNode.count("scale")) {
      parent->scale = jsonNode["scale"].get<std::vector<float>>();
    }
    if (jsonNode.count("matrix")) {
      parent->matrix = jsonNode["matrix"].get<std::vector<float>>();
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
    uint bufferViewId = accessor["bufferView"];

    const auto& bufferView = jsonBufferViews[bufferViewId];

    uint count = accessor["count"];
    uint componentType = accessor["componentType"].get<uint>();
    uint bytePerComponent = typeToSize[componentType];
    uint numComponents = typeTable[accessor["type"]];
    uint elementSize = numComponents * bytePerComponent;
    uint totalSize = elementSize * count;
    uint byteStride = (accessor.count("byteStride")) ?
                      accessor["byteStride"].get<uint>() : 0;

    uint bufferId = bufferView["buffer"].get<uint>();
    uint offset = accessor["byteOffset"].get<uint>() +
                  bufferView["byteOffset"].get<uint>();


    if (bufferView.count("target")) {

      uint target = bufferView["target"];
      if (target == glTFLoader::Target::ELEMENT_ARRAY_BUFFER &&
            byteStride != 0) {
        std::string error = "BufferView '" + std::to_string(bufferViewId);
        error += "': target is ELEMENT_ARRAY_BUFFER but byteStride isn't null.";
        this->logError(error);

        return;
      }

    }

    if (this->binaryFiles_.find(bufferId) == this->binaryFiles_.end()) {
      if (!registerBuffer(bufferId)) return;
    }

    uint8_t * rawData = this->binaryFiles_[bufferId];
    if (rawData == nullptr) return;

    if (sizeof(T) < bytePerComponent) {
      std::string error = "ComponentType: Bytes per component is ";
      error += std::to_string(bytePerComponent);
      error += " but loader expected at most " + std::to_string(sizeof(T));
      this->logError(error);

      return;
    }

    result.reserve(count);
    if ((byteStride == 0 || byteStride == elementSize) &&
          sizeof(T) == bytePerComponent) {
      copy(&rawData[0], &rawData[count], back_inserter(result));
    } else {
      for (size_t i = 0; i < count; ++i) {
        //memcpy(rawData + i, rawData + i * byteStride, elementSize);

        // result[i * byteStride]
      }
    }

    std::string debug = "\t-> Accessing buffer '" + std::to_string(bufferId);
    debug += "' > " + std::to_string(count);
    this->debug(debug);
  }

  bool
  glTFLoader::registerBuffer(uint bufferId) {

    const auto& jsonBuffers = this->json_["buffers"];
    const auto& jsonBuffer = jsonBuffers[bufferId];
    const auto& bufferUri = jsonBuffer["uri"].get<std::string>();

    std::string uri = this->assetsFolderPath_ + bufferUri;
    std::ifstream ifs(uri, std::ios::in | std::ios::binary | std::ios::ate);

    // TODO: Handle binary file not read
    if (ifs.fail()) {
      this->logMissingFile(uri);
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

  bool
  glTFLoader::checkValidity() {

    // Top level missing keys checks
    return checkMissingKey("scenes") && checkMissingKey("nodes") &&
           checkMissingKey("accessors") && checkMissingKey("bufferViews") &&
           checkMissingKey("buffers");

  }

  bool
  glTFLoader::checkMissingKey(const std::string& key) {

    if (!this->json_.count(key)) {
      std::string error = "MissingKey: '" + key;
      error += "' not found.";
      this->logError(error);

      return false;
    }

    return true;
  }

} // namespace tiny3Dloader

