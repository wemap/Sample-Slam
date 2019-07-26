#include "xpcf/module/ModuleFactory.h"
#include "PipeLineSlam.h"

#include "SolARModuleOpencv_traits.h"
#include "SolARModuleTools_traits.h"
#include "SolARModuleFBOW_traits.h"
#include "core/Log.h"

#define USE_FREE
#define ONE_THREAD 1

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;
using namespace SolAR::MODULES::OPENCV;
using namespace SolAR::MODULES::FBOW;
#ifndef USE_FREE
using namespace SolAR::MODULES::NONFREEOPENCV;
#endif
using namespace SolAR::MODULES::TOOLS;

namespace xpcf = org::bcom::xpcf;


//#define VIDEO_INPUT


// Declaration of the module embedding the Slam pipeline
XPCF_DECLARE_MODULE("da89a6eb-3233-4dea-afdc-9d918be0bd74", "SlamModule", "The module embedding a pipeline to estimate the pose based on a multithreaded Slam")

extern "C" XPCF_MODULEHOOKS_API xpcf::XPCFErrorCode XPCF_getComponent(const boost::uuids::uuid& componentUUID,SRef<xpcf::IComponentIntrospect>& interfaceRef)
{
    xpcf::XPCFErrorCode errCode = xpcf::XPCFErrorCode::_FAIL;
    errCode = xpcf::tryCreateComponent<SolAR::PIPELINES::PipelineSlam>(componentUUID,interfaceRef);

    return errCode;
}

XPCF_BEGIN_COMPONENTS_DECLARATION
XPCF_ADD_COMPONENT(SolAR::PIPELINES::PipelineSlam)
XPCF_END_COMPONENTS_DECLARATION
