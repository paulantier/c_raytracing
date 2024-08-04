
// Ce header contient uniquement des définitions de variables utilisées pour plusieurs header mais surtout pour le main.
// Ils permettent de régler les paramètres de création de l'image et de la création des différents objets
#define nbLancers 1 // nombre de lancers pour chaque pixel du viewport
#define nbRebondsMax 6 // nombre de rebonds max dans l'arbre de récursion

#define Nespace 1 // Indice du milieu (vitesse de la lumière dans le milieu / c)

// Dimensions de l'image
#define Vh 2000
#define Vw 2000

#define focale 1 // focale (distance entre caméra et viewport)

#define M_PI 3.14159265358979323846

// dimensions des images de texture (besoin de constantes pour définir des matrices constantes pour stocker l'image)
#define heightTextureSol 667
#define widthTextureSol 1000
#define heightTexturePlanete 512
#define widthTexturePlanete 1024

// paramètres de génération des sphères (il y a un programme de génération de sphères derriere la caméra pour tester la réfraction et ces paramètres entrent en compte)
#define nombre_de_spheres_principales 4
#define taille_grid_spheres_secondaires 0 // planètes derriere la caméra
#define nombre_de_planetes 1
// nombre_de_spheres = nombre_de_spheres_principales + taille_grid_spheres_secondaires^2 + nombre de planetes
#define nombre_de_spheres 5 // nombre de sphères au total

#define gridSizeThreads 2 // doit être un diviseur de Vh et Vw !!!!!!, nombre de sections horizontales et verticales de l'image pour le multithreading

// coordonnées du centre de la terre pour certains calculs.
#define xCentrePlanete 0
#define yCentrePlanete -2
#define zCentrePlanete -30
