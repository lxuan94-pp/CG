## Project Members

- Xuan Liu (xl94)
- Jungwoo Lee (jl40)



## Implementation Note

- The ray tracing algorithm involved and the algorithms for computing intersection and normal vectors of the object are all implemented according to the textbook method.
- Dark side of a geometry looks sharp because diffuse and specular are strictly removed when dot product of light and normal are 0.
- For the ellipsoids and cones, to compute the normal vectors for the deformed shape, the inverse transpose of the deformation matrix are applied. 



## Control

- Press n to enter object add mode.
- Press (1 ~ 5) to add object.
- Press any other key to exit object add mode.
- After adding object, user is still in object add mode.
- Press m while an object is selected to remove it from the scene.



## Test Scenes

- The following ray file contains the light effect test data that may be caused by different positions of light source and relative positions of different objects. You could test the corresponding file through Ctrl + 'o', or you could view the results according to the corresponding name in images folder.

### Front light test (test_all_frontlight.ray)

- All objects are illuminated by (5, 5, 5) light

### Back light test (test_all_backlight.ray)

- All objects are illuminated by (-5, -5, -5) light

### Double light test (test_all_doublelight.ray)

- All objects are illuminated by (5, 5 ,5) and (0, 5, 5) light to cast two shadows on the floor.

### All reflection test (test_all_reflection.ray)

- All objects are illuminated by (5, 5, 5) to show reflection and shadow on the box surface.

### Sphere reflection test (test_sphere_reflection.ray)

- Sphere displays recursive reflection of cylinder and cone

### Sphere refraction test (test_sphere_refraction.ray)

- Transparent sphere with refractive index (1.05) shows refraction of sphere

### Box reflection test (test_box_reflection.ray)

- Box displays recursive reflection of cylinder and cone

### Box refraction test (test_box_refraction.ray)

- Transparent box with refractive index (1.05) shows refraction of sphere

### Ellipsoid reflection test (test_ellipsoid_reflection.ray)

- Ellipsoid displays recursive reflection of cylinder and cone

### Ellipsoid refraction test (test_ellipsoid_refraction.ray)

- Transparent ellipsoid with refractive index (1.05) shows refraction of sphere

### Cylinder reflection test (test_cylinder_reflection.ray)

- Cylinder displays recursive reflection of cylinder and cone

### Cylinder refraction test (test_cylinder_refraction.ray)

- Transparent cylinder with refractive index (1.05) shows refraction of sphere

### Cone reflection test (test_cone_reflection.ray)

- Cone displays recursive reflection of cylinder and cone

### Cone refraction test (test_cone_refraction.ray)

- Transparent cone with refractive index (1.05) shows refraction of sphere



## Pretty Scenes

### Golden Egg (1.egg.ray)

The scene contains a large golden ellipsoid golden egg supported by a holder made of 8 black boxes placed on a reflective, opaque box floor. In front of the camera, there are three fully transparent jewels made of cone, cylinder, and box with refraction index of 1.05. The shadow of golden egg should be properly cast on the floor while the floor should show full reflection of golden egg, including reflection of three jewels. The primary purpose of this scene is to test refraction of box, cone, and cylinder.

### Castle (2.castle.ray)

The scene contains a toy castle made with reflective, semi-transparent boxes, cylinders, cones, and ellipsoids placed on a reflective floor along with a crystal sphere by the camera. The castle, made of multiple objects, correctly cast shadow and reflection on the surface. You can see refraction through boxes and even ellipsoids through cylinders. The purple tower on the right shows reflection of the center tower. Lastly, the crystal ball correctly displays the refraction of the castle.

### Snowman (3.snowman_and_house.ray)

The scene contains a snowman and a house with smoke curling upward from the chimneys. The snowman are composed of  8 different colors of spheres, 3 cones and two cylinders. The reflection could be seen from the head and body of the snowman. Also, the shadow cast on the snow which is made by a  grey  opaque box could be seen obviously. As for the house, the reason of the reflection on the snow which is different from the shadow of the snowman  is because the indexes assigned are not the same.  The refraction index of the objects are 1.6 and the refraction effect could be seen from the roof. And the smoke with high specular coefficient shows the miniature of the scene and the position of the light source. The purpose of this scene is to prove the correctness of all the light effects and the results of ray tracing on the all the objects that we are required to complete.

### Dog (4.dog_and_tree.ray)

The scene contains a  cute short-legged dog, a toy ball, two trees on the grass and a shining sun. There are 30 objects and two different positions of lights in the scene. The trees are composed of 3 cones with different radius and a brown opaque cylinder. The dog is composed of several boxes, ellipsoids and cylinders with  strong specular light effect which makes it bright and shining. The sun consists of three ellipsoids and a cylinder with reflection index of 0.1 which will not cause the strong reflected light, making the sun more real. The primary of this scene shows the correct specular light, shadow cast on the object under given two lights. Also, the reflection can be seen on the grass which is made by a large opaque green box with an appropriate reflection index. By showing the various light effects, it retains the vividness of the colors, too.

