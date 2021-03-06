# rob534_final_project
final project for rob534


3 enviroments

## Simple world 
* Arm can reach all sides of object.

        rosrun kinova_scripts kinova_path_planning.py 0
  
<image>
<img src="images/env_0.png" width="400">
</image>

## Moderate World
* Go around or over the top.
 

        rosrun kinova_scripts kinova_path_planning.py 1
        
<image>
<img src="images/env_1.png" width="400">
</image>



## Hard World
* Go around through opening.
 

        rosrun kinova_scripts kinova_path_planning.py 2
        
<image>
<img src="images/env_2.png" width="400">
</image>
