#pragma once

#define EPSILON 1e-6
#define ICP_RAND(min, max) ((min) + (double)rand()/(RAND_MAX+1)*((max)-(min)))
#define PTR_CONTROL_PARAMS (dynamic_cast<CControlParameters*>(hiveCommon::CProductFactoryData::getInstance()->getOrCreateProduct(CControlParameters::getClassSig())))