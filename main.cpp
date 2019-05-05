#include "denoiser.h"
#include <string>
#include <vector>
#include <utility>
#include <cmath>
#include <iostream>
#include <fstream>
#include <chrono>
#include<boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include"ptree.h"

using namespace std;
using boost::property_tree::ptree;
using std::string;
using std::vector;
using Error = std::pair<double, double>;

// void addNoise(DataManager *data, double noise_level);
int denoise(DataManager *data, int argc, char *argv[], ptree &pt);
int process(int argc, char **argv, ptree &pt);
Error estimateMSAE(DataManager &data);
template<typename T, typename U, class ...Types>
void setParameters(ParameterSet *param, T name, U value, Types... rest);
template<typename T, typename U>
void setParameters(ParameterSet *param, T name, U value);


//---------------------------------------------------------------------------------------------------
int main(int argc, char *argv[])
{
  //QCoreApplication a(argc, argv);

  ptree pt;
  try {
    zjucad::read_cmdline(argc, argv, pt); 
    if (0 != process(argc, argv, pt))
      cout << "# [error] mission failed" << endl;
    else 
      cout << "# [info] mission accomplished" << endl;
  } catch (const boost::property_tree::ptree_error &e) {
    cerr << "Usage: " << endl;
    zjucad::show_usage_info(cerr, pt);
  } catch (const exception &e) {
    cerr << "# " << e.what() << endl;
  }
  return 0;
}

template<typename T, typename U>
void setParameters(ParameterSet *param, T name, U value)
{
  param->setValue(name, value);
}

template<typename T, typename U, class ...Types>
void setParameters(ParameterSet *param, T name, U value, Types... rest)
{
  param->setValue(name, value);
  setParameters<T, U>(param, rest...);
}

Error estimateMSAE(DataManager &data)
{
  TriMesh original = data.getOriginalMesh();
  original.update_normals();
  TriMesh denoised_mesh = data.getDenoisedMesh();
  denoised_mesh.update_normals();
  TriMesh noisy_mesh = data.getNoisyMesh();
  noisy_mesh.update_normals();
  vector<TriMesh::Normal> original_normal, noisy_normal, denoised_normal;
  for(auto f_it = original.faces_begin(); f_it != original.faces_end(); f_it++)
  {
    TriMesh::Normal ni = original.normal(*f_it);
    original_normal.push_back(ni);
  }
  for(auto f_it = noisy_mesh.faces_begin(); f_it != noisy_mesh.faces_end(); f_it++)
  {
    TriMesh::Normal ni = noisy_mesh.normal(*f_it);
    noisy_normal.push_back(ni);
  }
  for(auto f_it = denoised_mesh.faces_begin(); f_it != denoised_mesh.faces_end(); f_it++)
  {
    TriMesh::Normal ni = denoised_mesh.normal(*f_it);
    denoised_normal.push_back(ni);
  }

  double noisy_error = 0.0, denoised_error = 0.0;
  for(decltype(original_normal.size()) sz = original_normal.size(), i = 0; i != sz; ++i)
  {
    denoised_error += acos(original_normal[i] | denoised_normal[i]);
    noisy_error += acos(original_normal[i] | noisy_normal[i]);
  }

  return Error(noisy_error / original_normal.size(), denoised_error / original_normal.size());
}


int process(int argc, char **argv, ptree &pt) {
  pt.put("method.desc", "method type");
  const int method_id = pt.get<int>("method.value");//stoi(argv[1]);
  pt.put("input.desc", "input obj file");
  const string model_file = pt.get<string>("input.value");//argv[2];
  auto model_name = model_file.substr(0, model_file.length() - 4); // assume the file end with .obj
  
  DataManager data;
  data.ImportMeshFromFile(model_file);
  
  if (0 == method_id) { // add noise
    // if (6 != argc) {
    //   cerr << "# [error] Usage: ./denoiser 0 input.obj noise_type noise_direction noise_level" << endl;
    //   return __LINE__;
    // }
    ParameterSet param_noise;
    Noise noiser(&data, &param_noise);
    
    pt.put("noise_type.desc", "noise type (gaussian 0)");
    if (!zjucad::has("noise_type.value", pt))
      pt.put("noise_type.value", 0);
    const int noise_type = pt.get<int>("noise_type.value"); //stoi(argv[3])
    param_noise.setValue(string("Noise type"), noise_type);
    
    pt.put("noise_direction.desc", "noise direction (normal 0, random 1)");
    if (!zjucad::has("noise_direction.value", pt))
      pt.put("noise_direction.value", 1);
    const int noise_direction = pt.get<int>("noise_direction.value"); //stoi(argv[4])
    param_noise.setValue(string("Noise direction"), noise_direction);

    pt.put("noise_level.desc", "noise level (0.1~1.0)");
    param_noise.setValue(string("Noise level"), pt.get<double>("noise_level.value")); //stod(argv[5])
    
    param_noise.setValue(string("Impulsive level"), 0.0);
    
    noiser.addNoise();
    param_noise.print();

    char file_name[100];
    sprintf(file_name, "%s_%s_%s_Level_%.2f.obj", model_name.c_str(),
            ((0 == noise_type)? "Gaussian" :"Other"),
            ((0 == noise_direction)? "Normal":"Random"),
            pt.get<double>("noise_level.value"));
    // string file_name = model_name + "_noise_" + "direct_" + argv[3]  + ".obj";
    data.ExportMeshToFile(file_name);
    cerr << "# [info] write file: " << file_name << endl;
  } else {
    
    // auto beg = std::chrono::high_resolution_clock::now();
    auto beg = clock();
    if (0 != denoise(&data, argc, argv, pt))
      return __LINE__;
    // auto end = std::chrono::high_resolution_clock::now();
    auto end = clock();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - beg).count();
    double duration = (end-beg)*1.0/CLOCKS_PER_SEC;
    cerr << "# [info] computation time: " << duration << "s" << endl; 
    
    auto er = estimateMSAE(data);
    // export the denoised model.
    char file_name[100];
    sprintf(file_name, "%s_method_%d.obj", model_name.c_str(), method_id);
    // data.ExportMeshToFile(model_name + "_" + string(method_id) + ".obj");
    data.ExportMeshToFile(file_name);
    cerr << "# [info] write file: " << file_name << endl;

    pt.put("time.desc", "time");
    pt.put("time.value", duration);
    // export measurement
    sprintf(file_name, "%s_method_%d.out", model_name.c_str(), method_id);
    std::ofstream output(file_name);
    for (ptree::const_iterator it = pt.begin(); it != pt.end(); ++it) {
      const string desc_path = it->first.data();
      output << desc_path << ": " << pt.get<string>(desc_path+".value") << endl;
    }
    // boost::property_tree::write_ini(file_name, pt);
    if (0) {
      std::ofstream output(file_name);
      output << "params: "; 
      for (int i = 3; i < argc; ++i) // output parameters 
        output << argv[i] << ' ';
      output << "\n";
      output << "Time: " << duration << endl;
      // output << er.first << " " << er.second <<  " " << duration << endl;;
      // duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - beg).count();
      output.close();
    }
    cerr << "# [info] write file: " << file_name << endl;
  }
  return 0;
}

// void addNoise(DataManager *data, double noise_level)
// {
//   ParameterSet param_noise;
//   Noise noiser(data, &param_noise);
//   param_noise.setValue(string("Noise level"), noise_level);
//   noiser.addNoise();
//   param_noise.print();
//   return ;
// }


int denoise(DataManager *data, int argc, char *argv[], ptree &pt)
{
  const int denoiser_type = pt.get<int>("method.value");
  // int denoiser_type = std::stoi(argv[1]);
  ParameterSet parameters;
  switch(denoiser_type){
    case 1:
      {
        BilateralMeshDenoising denoiser(data, &parameters);
        const int iteration_num = zjucad::get_ptree_item(pt, "iteration_num", "iteration number", 3);
        parameters.setValue("Iteration Num.", iteration_num); //std::stoi(argv[3])
        cerr << "# [info] denoise method: Bilateral Filtering" << endl;
        denoiser.denoise();
      }
      break;
    case 2:
      {
        BilateralNormalFilteringForMeshDenoising denoiser(data, &parameters);
        const int denoise_type = zjucad::get_ptree_item(pt, "denoise_type", "local 0, global 1", 0);
        parameters.setValue(string("Denoise Type"), denoise_type);
        const int face_neighbor = zjucad::get_ptree_item(pt, "face_neighbor", "vertex based 0, edge based 1", 0);
        parameters.setValue(string("Face Neighbor"),  face_neighbor);
        const double sigma_c = zjucad::get_ptree_item(pt, "sigma_c", "sigma c multiplier, 0.4-1.0", 1.0);
        parameters.setValue(string("Multiple(* sigma_c)"), sigma_c);
        const double sigma_s = zjucad::get_ptree_item(pt, "sigma_s", "sigma s, 0.1-0.6", 0.35);
        parameters.setValue(string("sigma_s"), sigma_s);
        const int normal_iteration_num = zjucad::get_ptree_item(pt, "normal_iteration_num", "normal iteration number", 20);
        parameters.setValue(string("Normal Iteration Num."), normal_iteration_num);
        const double smoothness = zjucad::get_ptree_item(pt, "smoothness", "smoothness", 0.01);
        parameters.setValue(string("Smoothness"), smoothness);
        const int vertex_iteration_num = zjucad::get_ptree_item(pt, "vertex_iteration_num", "vertex iteration number", 10);
        parameters.setValue(string("Vertex Iteration Num."), vertex_iteration_num);
        cerr << "# [info] denoise method: Bilateral Normal Filtering" << endl;
        denoiser.denoise();
      }
      break;
    case 3:
      {
        FastAndEffectiveFeaturePreservingMeshDenoising denoiser(data, &parameters);
        if (7 != argc) {
          cerr << "# [info ] Usage: ./denoiser 3 input.obj face_neighbor threshold_T normal_iteration vertex_iteration" <<endl;
          return __LINE__;
        }
        parameters.setValue(string("Face Neighbor"), std::stoi(argv[3]));
        parameters.setValue(string("Threshold T"), std::stod(argv[4]));
        parameters.setValue(string("Normal Iteration Num."), std::stoi(argv[5]));
        parameters.setValue(string("Vertex Iteration Num."), std::stoi(argv[6]));
        cerr << "# [info] denoise method: Fast And Effective Feature Preserving" << endl;
        denoiser.denoise();
      }
      break;
    case 4:
      {
        MeshDenoisingViaL0Minimization denoiser(data, &parameters);
        if (7 != argc){
          cerr <<" # [info] Usage: ./denoiser 4 input.obj mu_beta beta beta_max mu_alpha" << endl;
          return __LINE__;
        }
        parameters.setValue(string("mu_beta"), std::stod(argv[3]));
        parameters.setValue(string("beta"), std::stod(argv[4]));
        parameters.setValue(string("beta_max"), std::stod(argv[5]));
        parameters.setValue(string("mu_alpha"), std::stod(argv[6]));
        cerr << "# [info] denoise method: Mesh Denoising Via L0 Minimization" << endl;
        denoiser.denoise();
      }
      break;
    case 5:
      {
        NonIterativeFeaturePreservingMeshFiltering denoiser(data, &parameters);
        if (5 != argc) {
          cerr << "# [info] Usage: ./denoiser 5 input.obj sigma_f sigma_g" << endl;
          return __LINE__;
        }
        parameters.setValue(string("sigma_f/mean_edge_length"), std::stod(argv[3]));
        parameters.setValue(string("sigma_g/mean_edge_length"), std::stod(argv[4]));
        cerr << "# [info] denoise method: Non Iterative Feature Preserving" << endl;
        denoiser.denoise();
      }
      break;
    default:
      return __LINE__;
  }
  parameters.print();
  return 0;
}
