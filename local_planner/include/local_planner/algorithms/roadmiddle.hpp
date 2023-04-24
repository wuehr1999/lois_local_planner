#ifndef ALGO_ROADMIDDLE_HPP
#define ALGO_ROADMIDDLE_HPP

#include <local_planner/planningalgorithm.hpp>
#include <Python.h>

/**
 @brief Algorithm for controlling the robots position in the middle of the road.
 **/
class Algo_Roadmiddle : public PlanningAlgorithm
{
public:
  Algo_Roadmiddle(std::string name, int id, int idRecovery, PlannerData* dat, std::string coeffConfig = "home/lois/catkin_ws/src/planner/launch/middle.py"):PlanningAlgorithm(name, id, idRecovery, dat)
  {
    configPath = coeffConfig;
  }

  ~Algo_Roadmiddle()
  {
    Py_Finalize();
  }

  PlanningAlgorithm::Outcomes plan()
  {
   bool critical = false;
   initializeCoeffs();
   if(dat->noway)
   {
     return PlanningAlgorithm::Outcomes::RECOVERY;
   }
   else 
   {
     v = dat->config->vxMax;
//     w = dat->config->avg_p * dat->avg;
//     args = PyTuple_Pack(1, PyFloat_FromDouble(dat->avg));
//     result = PyObject_CallObject(lookup, args);
     w = PyFloat_AsDouble(result);
     if(w < -dat->config->wMax)
     {
       w = -dat->config->wMax;
     }
     else if(w > dat->config->wMax)
     {
       w = dat->config->wMax;
     }

     if((!critical && fabs(dat->avg) <= dat->config->avg_switch) ||
             (critical && fabs(dat->avg) <= dat->config->avg_hyst))
     {
       critical = false;
       return PlanningAlgorithm::Outcomes::UNCTRITICAL;
     }
     else
     {
       critical = true;
       return PlanningAlgorithm::Outcomes::CRITICAL;
     }
   }
 }

  void visualize(){}

private:
  std::string configPath;
  PyObject* lookup;
  PyObject* result;
  PyObject* it;
  PyObject* args;
  double thresholdCrash, thresholdNear;

  void initializeCoeffs()
  {
    static bool initialized = false;
    if(!initialized)
    {
      initialized = true;
      std::vector<std::string> elements;
      std::string path, file;
      std::stringstream ss(configPath);
      std::string item;
      while(getline(ss, item, '/'))
      {
        elements.push_back(item);
      }
      path = "/";
      for(int i = 0; i < elements.size() - 1; i++)
      {
        path += elements.at(i);
        if(i < elements.size() - 2)
        {
          path += "/";
        }
      }
      file = elements.at(elements.size() - 1);

      int ext = file.rfind(".py");
      if(ext != std::string::npos)
      {
        file.resize(ext);
      }
      Py_Initialize();
      PyRun_SimpleString("import sys;");
      std::string instruction = "sys.path.append('" + path + "')";
      PyRun_SimpleString(instruction.c_str());
      PyObject* modStr = PyString_FromString(file.c_str());
      PyObject* mod = PyImport_Import(modStr);
      lookup = PyObject_GetAttrString(mod, (char*)"speedLookup");
      PyObject* crashThresholdsFun = PyObject_GetAttrString(mod, (char*)"crashThresholds");

      result = PyObject_CallObject(crashThresholdsFun, NULL);
      it = PyList_GetItem(result, 0);
      thresholdCrash = PyFloat_AsDouble(it);
      it = PyList_GetItem(result, 1);
      thresholdNear = PyFloat_AsDouble(it);
    }
  }
};
#endif
