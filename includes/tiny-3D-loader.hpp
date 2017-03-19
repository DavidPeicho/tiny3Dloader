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

#include "json.hpp"

#include <iostream>
#include <fstream>
#include <string>

#include <unordered_map>

namespace tiny3Dloader {

  namespace scene {

    struct Primitive {
      std::vector<float>  vertices;
      std::vector<float>  normals;
      std::vector<float>  texcoords;
      std::vector<uint>   indices;
    };

    struct Mesh {
      ~Mesh();

      std::string             name;
      std::vector<Primitive*> primitives;
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
      ~Scene();

      std::string         name;
      std::vector<Node*>  nodes;
    };

    Mesh::~Mesh() {

      for (const auto& primitivePtr : primitives) {
        delete primitivePtr;
      }
      primitives.clear();

    }

    Node::~Node() {

      // The meshes are not freed from the node,
      // but rather from the map containing every meshes.
      meshes.clear();
      children.clear();

    }

    Scene::~Scene() {
      // The nodes are not freed from the scene,
      // but rather from the map containing every nodes.
      nodes.clear();
    }

  } // namespace scene

  class Loader {

    public:
      typedef std::vector<std::shared_ptr<scene::Scene>> ScenesList;

    public:
      virtual void
      load(const std::string& pathToFile,
           const std::string& assetsFolderPath,
           ScenesList& scenes) = 0;

      virtual void
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
      std::string                               assetsFolderPath_;
      std::string                               errorStr_;

      std::unordered_map<size_t, scene::Node*>  nodesMap_;
      std::unordered_map<size_t, scene::Mesh*>  meshesMap_;

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
      ELT_ARRAY_BUFFER = 34963,
    };

    public:
      typedef std::vector<std::shared_ptr<scene::Scene>> ScenesList;

    public:
      void
      load(const std::string& pathToFile,
           const std::string& assetsFolderPath,
           ScenesList& scenes) override;

      void
      freeScene() override;

    private:
      void
      processNode(uint nodeId);

      void
      processTransform(uint nodeId);

      void
      processMesh(uint nodeId, uint meshId);

      template <typename T>
      void
      processAccessor(const nlohmann::json& json, const char* key,
                      std::vector<T>& result);

      template <typename T>
      T
      extractData(uint8_t* buffer, size_t stride, size_t eltSize, size_t i);

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
      typedef std::vector<std::shared_ptr<scene::Scene>> ScenesList;

    public:
      ~Importer();

    public:
      bool
      load(const std::string& pathToFile, ScenesList& scenes);

      bool
      load(const std::string& pathToFile, const std::string& assetsFolderPath,
           ScenesList& scenes);

      void
      freeScene();

      inline void
      setDebug(bool flag) { this->debug_ = flag; }

    public:
      inline const std::string
      getError() const {

        if (loader_) return loader_->getError();
        return "";

      }

    private:
      std::shared_ptr<Loader> loader_ = nullptr;
      bool                    debug_  = false;

  };

  Importer::~Importer() {

    freeScene();

  }

  bool
  Importer::load(const std::string& pathToFile,
                 Importer::ScenesList& scenes) {

    return load(pathToFile, "", scenes);

  }

  bool
  Importer::load(const std::string& pathToFile,
                 const std::string& assetsFolderPath,
                 Importer::ScenesList& scenes) {

    std::string ext = pathToFile.substr(pathToFile.find_last_of(".") + 1);
    if (ext == "gltf") {
      this->loader_ = std::make_shared<glTFLoader>();
    } else {
      return false;
    }

    this->loader_->setDebug(this->debug_);
    this->loader_->load(pathToFile, assetsFolderPath, scenes);

    return this->loader_->getError().empty();
  }

  void
  Importer::freeScene() {

    if (!loader_) return;

    loader_->freeScene();
    loader_ = nullptr;

  }

  void
  Loader::freeScene() {

    this->debug("Begins memory freeing...");

    for (const auto& n : nodesMap_) {
      delete n.second;
      std::string debug = "\tNode '" + std::to_string(n.first) + "' freed";
      this->debug(debug);
    }
    for (const auto& m : meshesMap_) {
      delete m.second;
      std::string debug = "\tMesh '" + std::to_string(m.first) + "' freed";
      this->debug(debug);
    }
    nodesMap_.clear();
    meshesMap_.clear();

    this->debug("Ends memory freeing..");

  }

  void
  glTFLoader::load(const std::string& pathToFile,
                   const std::string& assetsFolderPath,
                   Loader::ScenesList& scenesResult) {

    std::ifstream stream(pathToFile);
    if (stream.fail()) {
      this->logMissingFile(pathToFile);
      return;
    }

    this->debug("Tiny3DLoader: Debug Trace");

    stream >> this->json_;

    this->assetsFolderPath_ = assetsFolderPath;

    if (!checkValidity()) return;

    uint nodeId = 0;
    auto& jsonNodes = this->json_["nodes"];

    // Allocates every nodes and add
    // them to the nodes cache.
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
        auto scene = std::make_shared<scene::Scene>();
        if (jsonScene.count("name")) scene->name = jsonScene["name"];

        const auto& nodes = jsonScene["nodes"];
        for (uint nodeId : nodes) {
          scene->nodes.push_back(nodesMap_[nodeId]);
        }

        scenesResult.push_back(scene);
      }
    }

    this->debug("Tiny3DLoader: End Trace");
  }

  void
  glTFLoader::freeScene() {


    for (const auto& elt : this->binaryFiles_) {
      delete[] elt.second;
    }
    binaryFiles_.clear();

    Loader::freeScene();

  }

  void
  glTFLoader::processNode(uint nodeId) {

    auto& jsonNode = this->json_["nodes"][nodeId];
    auto& node = nodesMap_[nodeId];
    node->name = (jsonNode.count("name")) ? jsonNode["name"] : "";

    debug("\tNode: " + std::to_string(nodeId) + " [" + node->name + "]");

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

    // Checks the meshes cache to avoid making
    // multiple allocations for the same mesh
    const auto& mapIt = this->meshesMap_.find(meshId);
    if (mapIt != this->meshesMap_.end()) {
      parentNode->meshes.push_back(mapIt->second);
      return;
    }

    auto& jsonMesh = this->json_["meshes"][meshId];
    auto& jsonPrimitives = jsonMesh["primitives"];

    this->debug("\t\tMesh: " + std::to_string(meshId));

    scene::Mesh* mesh = new scene::Mesh;
    for (const auto& jsonPrimitive : jsonPrimitives) {
      const auto &attributes = jsonPrimitive["attributes"];

      scene::Primitive* primitive = new scene::Primitive;

      processAccessor(attributes, "NORMAL", primitive->normals);
      processAccessor(attributes, "POSITION", primitive->vertices);

      // Processes all texcoords
      std::string texcoordKey = "TEXCOORD_0";
      size_t texcoordIdx = 0;
      while (attributes.count(texcoordKey.c_str())) {
        processAccessor(attributes, texcoordKey.c_str(), primitive->texcoords);
        texcoordKey = "TEXCOORD_" + std::to_string(++texcoordIdx);
      }

      // Processes primitive indexes
      processAccessor(jsonPrimitive, "indices", primitive->indices);

      mesh->primitives.push_back(primitive);
    }
    parentNode->meshes.push_back(mesh);
    this->meshesMap_[meshId] = mesh;
  }

  template <typename T>
  void
  glTFLoader::processAccessor(const nlohmann::json& json, const char* key,
                              std::vector<T>& result) {

    static std::map<std::string, glTFLoader::Type> TYPE_TABLE = {
      { "SCALAR", glTFLoader::Type::SCALAR },
      { "VEC2", glTFLoader::Type::VEC2 },
      { "VEC3", glTFLoader::Type::VEC3 },
      { "VEC4", glTFLoader::Type::VEC4 },
      { "MAT2", glTFLoader::Type::MAT2 },
      { "MAT3", glTFLoader::Type::MAT3 },
      { "MAT4", glTFLoader::Type::MAT4 },
    };

    static std::map<uint, uint> TYPE_TO_SIZE = {
      { ComponentType::BYTE, 1 },
      { ComponentType::UBYTE, 1 },
      { ComponentType::SHORT, 2 },
      { ComponentType::USHORT, 2 },
      { ComponentType::FLOAT, 4 }
    };

    if (!json.count(key)) return;

    const auto& jsonAccessors = this->json_["accessors"];
    const auto& jsonBufferViews = this->json_["bufferViews"];

    size_t accessorId = json[key];
    const auto& accessor = jsonAccessors[accessorId];
    uint bufferViewId = accessor["bufferView"];

    const auto& bufferView = jsonBufferViews[bufferViewId];

    size_t count = accessor["count"];
    size_t componentType = accessor["componentType"].get<uint>();
    size_t bytePerComponent = TYPE_TO_SIZE[componentType];
    size_t numComponents = TYPE_TABLE[accessor["type"]];
    size_t elementSize = numComponents * bytePerComponent;
    size_t totalSize = elementSize * count;
    size_t stride = accessor.count("byteStride") ? accessor["byteStride"]
                                                     .get<uint>() : 0;

    size_t bufferId = bufferView["buffer"].get<uint>();
    size_t offset = accessor["byteOffset"].get<uint>() +
                    bufferView["byteOffset"].get<uint>();


    if (bufferView.count("target")) {
      size_t target = bufferView["target"];
      if (target == glTFLoader::Target::ELT_ARRAY_BUFFER && stride != 0) {
        std::string error = "BufferView '" + std::to_string(bufferViewId);
        error += "': target is ELEMENT_ARRAY_BUFFER but byteStride isn't null.";
        logError(error);
        return;
      }
    }

    if (sizeof(T) < bytePerComponent) {
      std::string error = "ComponentType: Bytes per component is ";
      error += std::to_string(bytePerComponent);
      error += " but loader expected at most " + std::to_string(sizeof(T));
      this->logError(error);
      return;
    }

    if (this->binaryFiles_.find(bufferId) == this->binaryFiles_.end()) {
      if (!registerBuffer(bufferId)) return;
    }

    uint8_t * rawData = this->binaryFiles_[bufferId];
    if (rawData == nullptr) return;

    stride = (stride) ? stride : elementSize;
    result.reserve(count * numComponents);

    if ((stride == 0 || stride == elementSize) &&
          sizeof(T) == bytePerComponent) {
      T const* rawDataTyped = reinterpret_cast<const T*>(rawData + offset);
      copy(rawDataTyped, rawDataTyped + count * numComponents, back_inserter(result));
    } else {
      for (size_t i = 0; i < count * numComponents; ++i) {
        T val = extractData<T>(rawData + offset, stride, elementSize, i);
        result.push_back(val);
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

    if (ifs.fail()) {
      this->logMissingFile(uri);
      return false;
    }

    std::ifstream::pos_type fileSize = ifs.tellg();
    char* buffer = new char[fileSize];

    ifs.seekg(0, std::ios::beg);
    ifs.read(buffer, fileSize);

    if (!ifs) {
      std::string error = "buffer `" + bufferUri + "'";
      error += ": read failed.";
      logError(error);
      return false;
    }

    this->binaryFiles_[bufferId] = reinterpret_cast<uint8_t*>(buffer);
    return true;
  }

  template <typename T>
  T
  glTFLoader::extractData(uint8_t* buffer, size_t stride, size_t eltSize,
                          size_t i) {

    T val = T();
    memcpy(&val, buffer + i * stride, eltSize);

    return val;
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

