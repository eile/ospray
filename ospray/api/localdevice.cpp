#include "localdevice.h"
// #include "../fb/swapchain.h"
#include "../common/model.h"
#include "../common/data.h"
#include "../geometry/trianglemesh.h"
#include "../render/renderer.h"
#include "../camera/camera.h"
#include "../volume/volume.h"
#include "../render/loadbalancer.h"
#include "../common/material.h"
#include "../common/library.h"
#include "../texture/texture2d.h"
#include "../lights/light.h"
// embree stuff

namespace ospray {
  namespace api {

    LocalDevice::LocalDevice(int *_ac, const char **_av)
    {
      char *logLevelFromEnv = getenv("OSPRAY_LOG_LEVEL");
      if (logLevelFromEnv) 
        logLevel = atoi(logLevelFromEnv);
      else
        logLevel = 0;

      ospray::init(_ac,&_av);

      // initialize embree. (we need to do this here rather than in
      // ospray::init() because in mpi-mode the latter is also called
      // in the host-stubs, where it shouldn't.
      std::stringstream embreeConfig;
      if (debugMode)
        embreeConfig << " threads=1";
      rtcInit(embreeConfig.str().c_str());

      assert(rtcGetError() == RTC_NO_ERROR);
      TiledLoadBalancer::instance = new LocalTiledLoadBalancer;
    }


    OSPFrameBuffer 
    LocalDevice::frameBufferCreate(const vec2i &size, 
                                   const OSPFrameBufferMode mode)
    {
      // FrameBufferFactory fbFactory = NULL;
      // switch(mode) {
      // case OSP_RGBA_I8:
      //   fbFactory = createLocalFB_RGBA_I8;
      //   break;
      // default:
      //   AssertError("frame buffer mode not yet supported");
      // }
      
      FrameBuffer *fb = new LocalFrameBuffer<uint32>(size);
      fb->refInc();
      return (OSPFrameBuffer)fb;
      // SwapChain *sc = new SwapChain(swapChainDepth,size,fbFactory);
      // sc->refInc();
      // Assert(sc != NULL);
      // return (OSPFrameBuffer)sc;
    }
    

    /*! map frame buffer */
    const void *LocalDevice::frameBufferMap(OSPFrameBuffer _fb)
    {
      Assert(_fb != NULL);
      FrameBuffer *fb = (FrameBuffer *)_fb;
      if (fb->renderTask) {
        fb->renderTask->done.sync();
        fb->renderTask = NULL;
      }
      return fb->map();
      // SwapChain *sc = (SwapChain *)_fb;
      // return sc->map();
    }

    /*! unmap previously mapped frame buffer */
    void LocalDevice::frameBufferUnmap(const void *mapped,
                                       OSPFrameBuffer _fb)
    {
      Assert2(_fb != NULL, "invalid framebuffer");
      FrameBuffer *fb = (FrameBuffer *)_fb;
      fb->unmap(mapped);
      // SwapChain *sc = (SwapChain *)fb;
      // sc->unmap(mapped);
    }

    /*! create a new model */
    OSPModel LocalDevice::newModel()
    {
      Model *model = new Model;
      model->refInc();
      return (OSPModel)model;
    }
    
    /*! finalize a newly specified model */
    void LocalDevice::commit(OSPObject _object)
    {
      ManagedObject *object = (ManagedObject *)_object;
      Assert2(object,"null object in LocalDevice::commit()");
      object->commit();

      // hack, to stay compatible with earlier version
      Model *model = dynamic_cast<Model *>(object);
      if (model)
        model->finalize();
    }
    
    /*! add a new geometry to a model */
    void LocalDevice::addGeometry(OSPModel _model, OSPGeometry _geometry)
    {
      Model *model = (Model *)_model;
      Assert2(model,"null model in LocalDevice::finalizeModel()");

      Geometry *geometry = (Geometry *)_geometry;
      Assert2(geometry,"null geometry in LocalDevice::finalizeGeometry()");

      model->geometry.push_back(geometry);
    }

    /*! create a new data buffer */
    OSPTriangleMesh LocalDevice::newTriangleMesh()
    {
      TriangleMesh *triangleMesh = new TriangleMesh;
      triangleMesh->refInc();
      return (OSPTriangleMesh)triangleMesh;
    }

    /*! create a new data buffer */
    OSPData LocalDevice::newData(size_t nitems, OSPDataType format, void *init, int flags)
    {
      Data *data = new Data(nitems,format,init,flags);
      data->refInc();
      return (OSPData)data;
    }
    
    /*! assign (named) string parameter to an object */
    void LocalDevice::setString(OSPObject _object, const char *bufName, const char *s)
    {
      ManagedObject *object = (ManagedObject *)_object;
      Assert(object != NULL  && "invalid object handle");
      Assert(bufName != NULL && "invalid identifier for object parameter");
      object->findParam(bufName,1)->set(s);
    }

    /*! assign (named) string parameter to an object */
    void LocalDevice::setVoidPtr(OSPObject _object, const char *bufName, void *v)
    {
      ManagedObject *object = (ManagedObject *)_object;
      Assert(object != NULL  && "invalid object handle");
      Assert(bufName != NULL && "invalid identifier for object parameter");
      object->findParam(bufName,1)->set(v);
    }

    /*! assign (named) int parameter to an object */
    void LocalDevice::setInt(OSPObject _object, const char *bufName, const int f)
    {
      ManagedObject *object = (ManagedObject *)_object;
      Assert(object != NULL  && "invalid object handle");
      Assert(bufName != NULL && "invalid identifier for object parameter");

      object->findParam(bufName,1)->set(f);
    }
    /*! assign (named) float parameter to an object */
    void LocalDevice::setFloat(OSPObject _object, const char *bufName, const float f)
    {
      ManagedObject *object = (ManagedObject *)_object;
      Assert(object != NULL  && "invalid object handle");
      Assert(bufName != NULL && "invalid identifier for object parameter");

      object->findParam(bufName,1)->set(f);
    }

    /*! assign (named) vec3f parameter to an object */
    void LocalDevice::setVec3f(OSPObject _object, const char *bufName, const vec3f &v)
    {
      ManagedObject *object = (ManagedObject *)_object;
      Assert(object != NULL  && "invalid object handle");
      Assert(bufName != NULL && "invalid identifier for object parameter");

      object->findParam(bufName,1)->set(v);
    }
    /*! assign (named) vec3i parameter to an object */
    void LocalDevice::setVec3i(OSPObject _object, const char *bufName, const vec3i &v)
    {
      ManagedObject *object = (ManagedObject *)_object;
      Assert(object != NULL  && "invalid object handle");
      Assert(bufName != NULL && "invalid identifier for object parameter");

      object->findParam(bufName,1)->set(v);
    }

    /*! assign (named) data item as a parameter to an object */
    void LocalDevice::setObject(OSPObject _target, const char *bufName, OSPObject _value)
    {
      ManagedObject *target = (ManagedObject *)_target;
      ManagedObject *value  = (ManagedObject *)_value;

      Assert(target != NULL  && "invalid target object handle");
      Assert(bufName != NULL && "invalid identifier for object parameter");

      target->setParam(bufName,value);
    }

    /*! create a new renderer object (out of list of registered renderers) */
    OSPRenderer LocalDevice::newRenderer(const char *type)
    {
      Assert(type != NULL && "invalid render type identifier");
      Renderer *renderer = Renderer::createRenderer(type);
      if (!renderer) {
        if (ospray::debugMode) 
          throw std::runtime_error("unknown renderer type '"+std::string(type)+"'");
        else
          return NULL;
      }
      renderer->refInc();
      return (OSPRenderer)renderer;
    }

    /*! create a new geometry object (out of list of registered geometrys) */
    OSPGeometry LocalDevice::newGeometry(const char *type)
    {
      Assert(type != NULL && "invalid render type identifier");
      Geometry *geometry = Geometry::createGeometry(type);
      if (!geometry) return NULL;
      geometry->refInc();
      return (OSPGeometry)geometry;
    }

      /*! have given renderer create a new material */
    OSPMaterial LocalDevice::newMaterial(OSPRenderer _renderer, const char *type)
    {
      Assert2(type != NULL, "invalid material type identifier");

      // -------------------------------------------------------
      // first, check if there's a renderer that we can ask to create the material.
      //
      Renderer *renderer = (Renderer *)_renderer;
      if (renderer) {
        Material *material = renderer->createMaterial(type);
        if (material) {
          material->refInc();
          return (OSPMaterial)material;
        }
      }

      // -------------------------------------------------------
      // if there was no renderer, check if there's a loadable material by that name
      //
      Material *material = Material::createMaterial(type);
      if (!material) return NULL;
      material->refInc();
      return (OSPMaterial)material;
    }

    /*! create a new camera object (out of list of registered cameras) */
    OSPCamera LocalDevice::newCamera(const char *type)
    {
      Assert(type != NULL && "invalid camera type identifier");
      Camera *camera = Camera::createCamera(type);
      if (!camera) {
        if (ospray::debugMode) 
          throw std::runtime_error("unknown camera type '"+std::string(type)+"'");
        else
          return NULL;
      }
      camera->refInc();
      return (OSPCamera)camera;
    }

    /*! create a new volume object (out of list of registered volumes) */
    OSPVolume LocalDevice::newVolume(const char *type)
    {
      Assert(type != NULL && "invalid volume type identifier");
      Volume *volume = Volume::createVolume(type);
      if (!volume) {
        if (ospray::debugMode) 
          throw std::runtime_error("unknown volume type '"+std::string(type)+"'");
        else
          return NULL;
      }
      volume->refInc();
      return (OSPVolume)volume;
    }

    /*! create a new volume object (out of list of registered volume types) with data from a file */
    OSPVolume LocalDevice::newVolumeFromFile(const char *filename, const char *type)
    {
      Assert(type != NULL && "invalid volume type identifier");
      Assert(filename != NULL && "no file name specified for volume");
      Volume *volume = Volume::createVolume(filename, type);
      if (!volume) {
        if (ospray::debugMode)
          throw std::runtime_error("could not create volume of type '" + std::string(type) + "' from file '" + std::string(filename) + "'");
        else
          return NULL;
      }
      volume->refInc();
      return (OSPVolume)volume;
    }

    /*! have given renderer create a new Light */
    OSPLight LocalDevice::newLight(OSPRenderer _renderer, const char *type) {
      Renderer  *renderer = (Renderer *)_renderer;
      if (renderer) {
        Light *light = renderer->createLight(type);
        if (light) {
          light->refInc();
          return (OSPLight)light;
        }
      }

      //If there was no renderer try to see if there is a loadable light by that name
      Light *light = Light::createLight(type);
      if (!light) return NULL;
      light->refInc();
      return (OSPLight)light;
    }

    /*! create a new Texture2D object */
    OSPTexture2D LocalDevice::newTexture2D(int width, int height, OSPDataType type, void *data, int flags) {
      Assert(width > 0 && "Width must be greater than 0 in LocalDevice::newTexture2D");
      Assert(height > 0 && "Height must be greater than 0 in LocalDevice::newTexture2D");
      Texture2D *tx = Texture2D::createTexture(width, height, type, data, flags);
      if(tx) tx->refInc();
      return (OSPTexture2D)tx;
    }

    /*! load module */
    int LocalDevice::loadModule(const char *name)
    {
      std::string libName = "ospray_module_"+std::string(name);
      loadLibrary(libName);
      
      std::string initSymName = "ospray_init_module_"+std::string(name);
      void*initSym = getSymbol(initSymName);
      if (!initSym)
        //        throw std::runtime_error("could not find module initializer "+initSymName);
        return 1;
      void (*initMethod)() = (void(*)())initSym;
      if (!initMethod) 
        return 2;
      try {
        initMethod();
      } catch (...) {
        return 3;
      }
      return 0;
    }

    /*! call a renderer to render a frame buffer */
    void LocalDevice::renderFrame(OSPFrameBuffer _fb, 
                                  OSPRenderer    _renderer)
    {
      FrameBuffer *fb       = (FrameBuffer *)_fb;
      // SwapChain *sc       = (SwapChain *)_sc;
      Renderer  *renderer = (Renderer *)_renderer;
      // Model *model = (Model *)_model;

      Assert(fb != NULL && "invalid frame buffer handle");
      // Assert(sc != NULL && "invalid frame buffer handle");
      Assert(renderer != NULL && "invalid renderer handle");
      
      // FrameBuffer *fb = sc->getBackBuffer();
      renderer->renderFrame(fb);
      // WARNING: I'm doing an *im*plicit swapbuffers here at the end
      // of renderframe, but to be more opengl-conform we should
      // actually have the user call an *ex*plicit ospSwapBuffers call...
      // sc->advance();
    }

    //! release (i.e., reduce refcount of) given object
    /*! note that all objects in ospray are refcounted, so one cannot
      explicitly "delete" any object. instead, each object is created
      with a refcount of 1, and this refcount will be
      increased/decreased every time another object refers to this
      object resp releases its hold on it; if the refcount is 0 the
      object will automatically get deleted. For example, you can
      create a new material, assign it to a geometry, and immediately
      after this assignation release its refcount; the material will
      stay 'alive' as long as the given geometry requires it. */
    void LocalDevice::release(OSPObject _obj)
    {
      if (!_obj) return;
      ManagedObject *obj = (ManagedObject *)_obj;
      obj->refDec();
    }

    //! assign given material to given geometry
    void LocalDevice::setMaterial(OSPGeometry _geometry, OSPMaterial _material)
    {
      Geometry *geometry = (Geometry*)_geometry;
      Material *material = (Material*)_material;
      geometry->setMaterial(material);
    }

  }
}

