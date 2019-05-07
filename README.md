# Denoiser
A command line mesh denoiser
This project is built by CMake
### Dependencies
OpenMesh library

### How to use it?
usage:
  denoiser 0 <path to the model> <level of noise>
  The output of the program is a noisy model (.obj)
  
  denoiser <type of denoiser> <path to the model> [parameters for denoiser]
  The output of the program is a denoised model (.obj) and a file of summary info (.out). 

  if some of [parameters for denoiser] is lost, those parameters would be set to default parameters.


| command | types of denoiser                              |
| ------- | ---------------------------------------------- |
| 0       | Add noise to model                             |
| 1       | BilateralMeshDenoising                         |
| 2       | BilateralNormalFiltering                       |
| 3       | FastAndEffectiveFeaturePreservingMeshDenoising |
| 4       | MeshDenoisingViaL0Minimization                 |
| 5       | NonIterativeFeaturePreservingMeshFiltering     |
| 6       | GuidedMeshNormalFiltering     |

For example:

#### Generate noisy data
./denoiser method=0 input=cylinder.obj noise_direction=0 noise_level=0.3
##### Parameter setting
* noise_direction: {0, 1}
* noise_level: {0.1 0.2, 0.3, ... 0.5}

#### Bilateral Filtering
./denoiser method=1 input=cylinder_Gaussin_Normal_level_0.10.obj iteration_num=5
##### Parameter setting
* iteration_num: {3, 5, 10, 15}

#### Bilateral Normal Filtering
./denoiser method=2 input=cylinder_Gaussin_Normal_level_0.10.obj  mu_sigma_c=1.0 sigma_s=0.35  normal_iteration_num=20  vertex_iteration_num=10
##### Parameter setting
* sigma_c: {0.6, 0.8, 1.0, 1.2}
* sigma_s: {0.1 0.3 0.35 0.4 0.5} 
* normal_iteration_num: {3, 5, 10, 15, 20,25}
* vertex_iteration_num: {3, 5, 10, 15, 20, 25}

#### FastAndEffectiveFeaturePreserving
./denoiser method=3 input=cylinder_Gaussin_Normal_level_0.10.obj  thd_T=0.5 face_neighbor=0 normal_iteration_num=20 vertex_iteration_num=50
##### Parameter setting
* normal_iteration_num: {10, 20, 30, 40}
* vertex_iteration_num: {20, 30, 40, 50, 60, 70}

#### L0Minimization
./denoiser method=4 input=cylinder_Gaussin_Normal_level_0.10.obj mu_beta=1.414 mu_alpha=0.5 beta=0.001 max_beta=1000
##### Parameter setting
* mu_beta:
* mu_alpha:
* max_beta: {100, 500, 1000, 1500}

#### NonIterativeFeaturePreserving
./denoiser method=5 input=cylinder_Gaussin_Normal_level_0.10.obj mu_sigma_f=1.0 mu_sigma_g=1.0
##### Parameter setting
* mu_sigma_f:
* mu_sigma_g:

#### GuidedMeshNormalFiltering
 ./denoiser method=6 input=cylinder.obj  use_central_face=true normal_iteration_num=20 denoise_type=0 face_neighbor=0 vertex_iteration_num=10 mu_face_dist=2 mu_sigma_s=1  sigma_r=0.35 smoothness=0.01
##### Parameter setting
* mu_sigma_s:
* sigma_r:
* normal_iteration_num:
* vertex_iteration_num:


