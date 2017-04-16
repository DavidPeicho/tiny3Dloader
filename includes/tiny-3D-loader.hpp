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

/*
 *
 * Usage:
 *
 * #define TINY3DLOADER_EXCEPTIONS // Required if exceptions are allowed
 * #include "tiny-3D-loader.hpp"
 *
 */

#pragma once

#include "json.hpp"

#include <iostream>
#include <fstream>
#include <string>

#include <unordered_map>

// Exceptions are disabled by default.
// You can easily enable them by using the following macro.
#if defined(TINY3DLOADER_EXCEPTIONS)
  #define TINY3D_THROW(exception) throw exception
  #define TINY3D_TRY try
  #define TINY3D_CATCH(exception) catch(exception)
#else
  #define TINY3D_THROW(exception) /* empty */
  #define TINY3D_TRY if (true)
  #define TINY3D_CATCH(exception) if (false)
#endif

namespace tiny3Dloader {

#define IS_SPACE(x) (((x) == ' ') || ((x) == '\t'))

// Handles getLine with various end of line character.
// See:
// http://stackoverflow.com/questions/6089231/getting-std-ifstream-to-handle-lf-cr-and-crlf
static inline std::istream&
safeGetline(std::istream& is, std::string& t) {
  t.clear();

  // The characters in the stream are read one-by-one using a std::streambuf.
  // That is faster than reading them one-by-one using the std::istream.
  // Code that uses streambuf this way must be guarded by a sentry object.
  // The sentry object performs various tasks,
  // such as thread synchronization and updating the stream state.

  std::istream::sentry se(is, true);
  std::streambuf* sb = is.rdbuf();

  for(;;) {
    int c = sb->sbumpc();
    switch (c) {
      case '\n':
        return is;
      case '\r':
        if(sb->sgetc() == '\n')
          sb->sbumpc();
        return is;
      case EOF:
        // Also handle the case when the last line has no line ending
        if(t.empty())
          is.setstate(std::ios::eofbit);
        return is;
      default:
        t += (char)c;
    }
  }
}

static inline std::string&
leftTrim(std::string& s, const char* t = " \t\n\r\f\v") {
  s.erase(0, s.find_first_not_of(t));
  return s;
}

static inline std::string&
rightTrim(std::string& s, const char* t = " \t\n\r\f\v") {
  s.erase(s.find_last_not_of(t) + 1);
  return s;
}

static inline std::string&
leftRightTrim(std::string& s, const char* t = " \t\n\r\f\v") {
  return rightTrim(leftTrim(s));
}

static inline size_t
countTokens(std::string str) {

  if (str.size() == 0) return 0;

  size_t count = 0;
  for (size_t i = 0; i < str.size() - 1; ++i) {
    if (!IS_SPACE(str[i]) &&
      (IS_SPACE(str[i + 1]) || i + 1 == str.size() - 1)) {
      ++count;
    }
  }
  return count;

}

static inline std::string
nextToken(std::string& line, const char* anyOf = " \t\r\n") {

  std::size_t beginIdx = line.find_first_not_of(anyOf);
  if (beginIdx == std::string::npos) {
    line.clear();
    return std::string("");
  }

  std::size_t idx = line.find_first_of(anyOf, beginIdx);
  std::string result;
  if (idx == std::string::npos) {
    result = line.substr(beginIdx, line.size() - beginIdx);
    line.clear();
  } else {
    result = line.substr(beginIdx, idx - beginIdx);
    line.erase(0, idx);
  }

  return result;

}

static inline bool
isNumber(std::string& token) {

  size_t i = 0;
  if (token[0] == '-') {
    ++i;
  }
  if (token[i] == '.') return false;

  bool pointFound = false;
  for ( ; i < token.size(); ++i) {
    if (token[i] != '.' && !std::isdigit(token[i])) {
      return false;
    } else if (token[i] == '.') {
      if (pointFound) return false;
      pointFound = true;
    }
  }

}

namespace exceptions {

class Tiny3DLoaderException : public std::exception {

  public:
    Tiny3DLoaderException(const std::string &msg) {

      this->msg_ = "Tiny3DLoaderException: " + msg;

    }

    virtual const char *what() const noexcept {

      return msg_.c_str();

    }

  protected:
    std::string msg_;

};

class MissingFileError : public Tiny3DLoaderException {

  public:
    MissingFileError(const std::string& fileName)
      : Tiny3DLoaderException("MissingFileError: " + fileName + " not found.") {
    }

};

class InvalidFileError : public Tiny3DLoaderException {

  public:
    InvalidFileError(const std::string& fileName)
      : Tiny3DLoaderException("InvalidFileError: " + fileName + " parse error.") {
    }

};

class LogicError : public Tiny3DLoaderException {

  public:
    LogicError(const std::string& msg)
      : Tiny3DLoaderException("LogicError: " + msg) {
    }

};

}

namespace scene {

struct Primitive {

  std::vector<float>        vertices;
  std::vector<float>        normals;
  std::vector<float>        texcoords;
  std::vector<unsigned int> indices;

};

struct Mesh {

  ~Mesh() {

    for (const auto& primitivePtr : primitives) {
      delete primitivePtr;
    }
    primitives.clear();

  }

  std::string             name;
  std::vector<Primitive*> primitives;

};

struct Node {

  ~Node() {

    // The meshes are not freed from the node,
    // but rather from the map containing every meshes.
    meshes.clear();
    children.clear();

  }

  std::string         name;
  std::vector<Mesh*>  meshes;
  std::vector<Node*>  children;

  std::vector<float>  translation;
  std::vector<float>  rotation;
  std::vector<float>  scale;
  std::vector<float>  matrix;

};

struct Scene {

  ~Scene() {

    // The nodes are not freed from the scene,
    // but rather from the map containing every nodes.
    nodes.clear();

  }

  std::string         name;
  std::vector<Node*>  nodes;

};

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
    freeScene() {

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

    inline void
    setDebug(bool debug) {

      debug_ = debug;

    }

  public:
    inline const std::string&
    getError() const {

      return errorStr_;

    }

  protected:
    inline void
    debug(const std::string& message) {

      if (debug_) std::cout << message << std::endl;

    };

    inline void
    logError(const std::string& errorMsg) {

      errorStr_ += "Tiny3DLoader: Error: ";
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

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                glTF Loader                                 //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

class glTFLoader : public Loader {

  public:
    typedef std::vector<std::shared_ptr<scene::Scene>> ScenesList;

  private:
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
    void
    load(const std::string& pathToFile,
         const std::string& assetsFolderPath,
         ScenesList& scenesResult) override {

      std::ifstream stream(pathToFile);
      if (stream.fail()) {
        TINY3D_THROW(exceptions::MissingFileError(pathToFile));
        this->logMissingFile(pathToFile);
        return;
      }

      this->debug("Tiny3DLoader: Debug Trace");

      stream >> this->json_;

      this->assetsFolderPath_ = assetsFolderPath;

      // In case exceptions are activated, this raises an exception at the
      // first mandatory key not found.
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
    freeScene() override {

      for (const auto& elt : this->binaryFiles_) {
        delete[] elt.second;
      }
      binaryFiles_.clear();

      Loader::freeScene();

    }

  private:
    void
    processNode(uint nodeId) {

      auto& jsonNode = this->json_["nodes"][nodeId];
      auto& node = nodesMap_[nodeId];
      node->name = (jsonNode.count("name")) ? jsonNode["name"] : "";

      debug("\tNode: " + std::to_string(nodeId) + " [" + node->name + "]");

      if (jsonNode.count("mesh")) {
        processMesh(nodeId, jsonNode["mesh"]);
      }

      processTransform(nodeId);

      if (jsonNode.count("children") == 0) return;

      for (const auto childId : jsonNode["children"]) {
        const auto& it = this->nodesMap_.find(childId);
        if (this->nodesMap_.find(childId) != this->nodesMap_.end()) {
          node->children.push_back(it->second);
        }
      }

    }

    void
    processTransform(uint nodeId) {

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
    processMesh(uint nodeId, uint meshId) {

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
    processAccessor(const nlohmann::json& json, const char* key,
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
          TINY3D_THROW(exceptions::LogicError(error));
          logError(error);
          return;
        }
      }

      if (sizeof(T) < bytePerComponent) {
        std::string error = "ComponentType: Bytes per component is ";
        error += std::to_string(bytePerComponent);
        error += " but loader expected at most " + std::to_string(sizeof(T));
        TINY3D_THROW(exceptions::LogicError(error));
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

    template <typename T>
    T
    extractData(uint8_t* buffer, size_t stride, size_t eltSize, size_t i) {

      T val = T();
      memcpy(&val, buffer + i * stride, eltSize);

      return val;

    }

    bool
    registerBuffer(uint bufferId) {

      const auto& jsonBuffers = this->json_["buffers"];
      const auto& jsonBuffer = jsonBuffers[bufferId];
      const auto& bufferUri = jsonBuffer["uri"].get<std::string>();

      std::string uri = this->assetsFolderPath_ + bufferUri;
      std::ifstream ifs(uri, std::ios::in | std::ios::binary | std::ios::ate);

      if (ifs.fail()) {
        TINY3D_THROW(exceptions::MissingFileError(bufferUri));
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

    bool
    checkValidity() {

      // Top level missing keys checks
      return checkMissingKey("scenes") && checkMissingKey("nodes") &&
             checkMissingKey("accessors") && checkMissingKey("bufferViews") &&
             checkMissingKey("buffers");

    }

    bool
    checkMissingKey(const std::string& key) {

      if (!this->json_.count(key)) {
        std::string error = "MissingKey: '" + key;
        error += "' not found.";
        TINY3D_THROW(exceptions::LogicError(error));
        this->logError(error);
        return false;
      }

      return true;

    }

  private:
    nlohmann::json                      json_;
    std::unordered_map<uint, uint8_t*>  binaryFiles_;

};

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                obj Loader                                  //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

class OBJLoader : public Loader {

  public:
    OBJLoader() : requestPrimitive_{true}
    { }

  public:
    void
    load(const std::string& pathToFile,
         const std::string& assetsFolderPath,
         Loader::ScenesList& scenes) override {

      std::ifstream ifs(pathToFile, std::ios::in);

      if (ifs.fail()) {
        this->logMissingFile(pathToFile);
        return;
      }

      // Wavefront .obj file format is different from the structure
      // exposed by the Tiny3DLoader library.
      // .obj objects are considered to be Tiny3DLoader nodes.
      // .obj groups are considered to be Tiny3DLoader meshes with one primitive.
      auto mesh = new scene::Mesh;
      auto node = new scene::Node;

      //mesh->primitives.push_back(prim);
      node->meshes.push_back(mesh);

      this->meshesMap_[0] = mesh;
      this->nodesMap_[0] = node;

      auto s = std::make_shared<scene::Scene>();
      scenes.push_back(s);

      std::string line;
      while (ifs.peek() != -1) {
        safeGetline(ifs, line);
        leftTrim(line);

        // Handles empty and commented lines
        if (line.size() <= 3 || line[0] == '#') continue;

        parseLine(line, s);
      }

      // Add final node
      s->nodes.push_back(this->nodesMap_[this->nodesMap_.size() - 1]);

    }

    void
    freeScene() override {

      Loader::freeScene();

    }

  private:
    void
    parseLine(std::string& line, std::shared_ptr<scene::Scene>& scene) {

      scene::Node* currNode = this->nodesMap_[this->nodesMap_.size() - 1];
      scene::Mesh* currMesh = this->meshesMap_[this->meshesMap_.size() - 1];

      switch (line[0]) {
        // Parses object, representing a Node
        // in the Tiny3DLoader data structure.
        case 'o': {
          if (currMesh->primitives.size()) {
            currNode = new scene::Node;
            currMesh = new scene::Mesh;
            this->nodesMap_[this->nodesMap_.size()] = currNode;
            this->meshesMap_[this->meshesMap_.size()] = currMesh;
            currNode->meshes.push_back(currMesh);
            scene->nodes.push_back(currNode);
            requestPrimitive_ = true;
          }
          line[0] = ' ';
          currNode->name = leftTrim(line);
          break;
        }
          // Parses group, representing a Mesh and a primitive
          // in the Tiny3DLoader data structure.
        case 'g': {
          if (currMesh->primitives.size()) {
            currMesh = new scene::Mesh;
            currNode->meshes.push_back(currMesh);
            this->meshesMap_[this->meshesMap_.size()] = currMesh;
            requestPrimitive_ = true;
          }
          line[0] = ' ';
          currMesh->name = leftTrim(line);
          break;
        }
          // Parses every data related to vertices:
          // vertices, normals, texcoords...
        case 'v': {
          if (IS_SPACE(line[1])) {
            line[0] = ' ';
            parseFloat3(line, vertices_);
          } else if (line[1] == 'n' && IS_SPACE(line[2])) {
            line[0] = ' ';
            line[1] = ' ';
            parseFloat3(line, normals_);
          } else if (line[1] == 't' && IS_SPACE(line[2])) {
            line[0] = ' ';
            line[1] = ' ';
            parseFloat2(line, texcoords_);
          }
          break;
        }
          // Parses faces by linking previously parsed data into
          // a Tiny3DLoader mesh.
        case 'f': {
          if (requestPrimitive_) {
            scene::Primitive* currPrimitive = new scene::Primitive;
            currMesh->primitives.push_back(currPrimitive);
            requestPrimitive_ = false;
            this->idx_ = 0;
          }
          if (IS_SPACE(line[1])) {
            line[0] = ' ';
            parseFace(line, currMesh);
          }
          break;
        }
          // Specifies a new material.
          // Only one material can be applied to a Tiny3DLoader primitive.
        case 'u': {
          if (line.compare(0, 6, "usemtl") == 0) {
            requestPrimitive_ = true;
          }
        }
      }

    }

    void
    parseFace(std::string& line, scene::Mesh* mesh) {

      size_t nbVertices = countTokens(line);
      if (nbVertices == 0) return;

      // Currently, Tiny3DLoader does not handle n-gons,
      // it only deals with triangle and quads.
      if (nbVertices > 4) {
        this->logError("Face: only triangles and quads supported.");
        return;
      }

      std::vector<std::string> facesToken;
      facesToken.reserve(nbVertices);
      while (!leftTrim(line).empty()) {
        facesToken.push_back(nextToken(line));
      }

      auto& currPrimitive = mesh->primitives[mesh->primitives.size() - 1];
      auto& primIndices = currPrimitive->indices;

      // Face is already triangulated
      if (nbVertices == 3) {
        for (auto& f : facesToken) {
          primIndices.push_back(parseFaceTriplet(f, currPrimitive));
        }
        return;
      }

      // Face is quadrangulated, we have to triangulate it.
      // First triangle.
      primIndices.push_back(parseFaceTriplet(facesToken[1], currPrimitive));
      primIndices.push_back(parseFaceTriplet(facesToken[3], currPrimitive));
      primIndices.push_back(parseFaceTriplet(facesToken[0], currPrimitive));
      // Second triangle.
      primIndices.push_back(parseFaceTriplet(facesToken[1], currPrimitive));
      primIndices.push_back(parseFaceTriplet(facesToken[2], currPrimitive));
      primIndices.push_back(parseFaceTriplet(facesToken[3], currPrimitive));

    }

    unsigned int
    parseFaceTriplet(std::string& face, scene::Primitive* p) {

      auto& primVertices = p->vertices;
      auto& primNormals = p->normals;
      auto& primTexcoords = p->texcoords;

      size_t vId = 0;
      size_t vnId = 0;
      size_t vtId = 0;

      // Optimization: Avoid having one index per vertex.
      // The technique here is to save each face string 'v0/vn0/vt0'
      // in a unique map, associated to a the real index later
      // provided to glDrawElements.
      auto hashIt = hashed_.find(face);
      if (hashIt != hashed_.end()) {
        return hashIt->second;
      }

      unsigned int currIdx = this->idx_;

      // The face string was not previously added to the hashed map,
      // we can now add it, and create a new element index.
      this->hashed_[face] = currIdx;
      this->idx_++;

      // Extracts the wavefront obj vertex id given in a face
      const char* ptr = face.c_str();

      vId = getRelativeIdx(std::atoi(ptr), vertices_.size() / 3);
      primVertices.push_back(vertices_[vId * 3]);
      primVertices.push_back(vertices_[vId * 3 + 1]);
      primVertices.push_back(vertices_[vId * 3 + 2]);

      ptr = ptr + strcspn(ptr, "/ \t\r\n");
      // f v1 v2 v3 ...
      if (ptr[0] != '/') return currIdx;

      // f v1//vn1 v2//vn2
      if ((++ptr)[0] == '/') {
        vnId = getRelativeIdx(std::atoi(ptr + 1), normals_.size() / 3);
        primNormals.push_back(normals_[vnId * 3]);
        primNormals.push_back(normals_[vnId * 3 + 1]);
        primNormals.push_back(normals_[vnId * 3 + 2]);
        return currIdx;
      }

      // f v1/vt1 v2/vt2
      const char* last = ptr + strcspn(ptr, "/ \t\r\n");
      vtId = getRelativeIdx(std::atoi(ptr + 1), texcoords_.size() / 3);
      primTexcoords.push_back(texcoords_[vtId * 3]);
      primTexcoords.push_back(texcoords_[vtId * 3 + 1]);

      if (last[0] != '/') return currIdx;

      // f v1/vt1/vn1 v2/vt2/vn2
      vnId = getRelativeIdx(std::atoi(ptr + 1), normals_.size() / 3);
      primNormals.push_back(normals_[vnId * 3]);
      primNormals.push_back(normals_[vnId * 3 + 1]);
      primNormals.push_back(normals_[vnId * 3 + 2]);

      return currIdx;

    }

    void
    parseFloat3(std::string& line, std::vector<float>& container) {

      auto xStr = nextToken(line);
      auto yStr = nextToken(line);
      auto zStr = nextToken(line);

      if (!parseFloat(xStr, container) || !parseFloat(yStr, container) ||
          !parseFloat(zStr, container)) {
        std::string error = "Float: parse error on line '" + line + "'";
        this->logError(error);
      }

    }

    void
    parseFloat2(std::string& line, std::vector<float>& container) {

      auto xStr = nextToken(line);
      auto yStr = nextToken(line);

      if (!parseFloat(xStr, container) || !parseFloat(yStr, container)) {
        std::string error = "Float: parse error on line '" + line + "'";
        this->logError(error);
      }

    }

    bool
    parseFloat(std::string& token, std::vector<float>& container) {

      if (isNumber(token)) return false;

      float val = std::stof(token);
      container.push_back(val);

      return true;

    }

    size_t
    getRelativeIdx(int idx, size_t size) {

      if (idx > 0) return idx - 1;

      return idx + size;

    }

  private:
    std::vector<float>                    vertices_;
    std::vector<float>                    normals_;
    std::vector<float>                    texcoords_;

    std::unordered_map<std::string, uint> hashed_;

    unsigned int                          idx_ = 0;
    bool                                  requestPrimitive_;

};

class Importer {

  public:
    typedef std::vector<std::shared_ptr<scene::Scene>> ScenesList;

  public:
    ~Importer() {

      freeScene();

    }

  public:
    bool
    load(const std::string& pathToFile, ScenesList& scenes) {

      return load(pathToFile, "", scenes);

    }

    bool
    load(const std::string& pathToFile, const std::string& assetsFolderPath,
         ScenesList& scenes) {

      std::string ext = pathToFile.substr(pathToFile.find_last_of(".") + 1);
      if (ext == "gltf") {
        this->loader_ = std::make_shared<glTFLoader>();
      } else if (ext == "obj") {
        this->loader_ = std::make_shared<OBJLoader>();
      } else {
        std::string error = "unsupported extension in '" + pathToFile + "`";
        TINY3D_THROW(exceptions::Tiny3DLoaderException(error));
        std::cerr << "Tiny3DLoader: Importer: " << error << std::endl;
        return false;
      }

      this->loader_->setDebug(this->debug_);
      this->loader_->load(pathToFile, assetsFolderPath, scenes);

      return this->loader_->getError().empty();

    }

    void
    freeScene() {

      if (!loader_) return;

      loader_->freeScene();
      loader_ = nullptr;

    }

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

} // namespace tiny3Dloader
