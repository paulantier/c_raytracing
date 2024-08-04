#ifndef CREATEENV_H
#define CREATEENV_H

#include "generationParameters.h"
#include "math.h"

// définition des tableaux contenants les objets Planete Sphere et Plan, ainsi que leurs caractéristiques telles que
// l'absorption lumineuse l'indice de milieu le type de réflexion, etc
double Planete[9];
double Spheres[nombre_de_spheres][9];
double eqPlan[4];
double APlan[3];
double Gsphere;
double Nsphere;
double ecartementSpheres=3;
double RGridSpheres=1;
double GGridSpheres=10;
double NGridSpheres=-1;

// On créer l'équation du plan
void creerSol() {
    eqPlan[0] = 0;
    eqPlan[1] = 1;
    eqPlan[2] = 0;
    eqPlan[3] = -20;

    // Obsolète depuis la création de la texture de sol :
    APlan[0] = 0;
    APlan[1] = 0.5;
    APlan[2] = 1;
}

void creerPlanete() {
    // Sphere 0 : terre
    Planete[0] = 11;    //rayon
    Planete[1] = xCentrePlanete;    //centreX
    Planete[2] = yCentrePlanete;   //centreY
    Planete[3] = zCentrePlanete;     //centreZ
    Planete[4] = 0;      //absorptionR
    Planete[5] = 0;      //absorptionG
    Planete[6] = 0;    //absorptionB
    Planete[7] = -1;   //glossiness
    Planete[8] = -1;    //refraction
}

void creerSpheres(double t) {
    // Sphere 1 : Très brillante en verre
    Spheres[0][0] = 0.5;    //rayon
    Spheres[0][1] = t*sin(t*4*M_PI/100)/100; //0.3;    //centreX
    Spheres[0][2] = t*cos(t*4*M_PI/100)/100; //0.5;   //centreY
    Spheres[0][3] = -2;     //centreZ
    Spheres[0][4] = 0;      //absorptionR
    Spheres[0][5] = 0;      //absorptionG
    Spheres[0][6] = 0;    //absorptionB
    Spheres[0][7] = 0.5;   //glossiness
    Spheres[0][8] = 1.5;    //refraction

    // Sphere 2 : Brillant Plexi Vert
    Spheres[1][0] = 1;
    Spheres[1][1] = -2.5;
    Spheres[1][2] = 0.7;
    Spheres[1][3] = -8;
    Spheres[1][4] = 0.5;
    Spheres[1][5] = 0;
    Spheres[1][6] = 1;
    Spheres[1][7] = 0.3;
    Spheres[1][8] = -1;

    // Sphere 3 : Mate rose
    Spheres[2][0] = 0.3;
    Spheres[2][1] = 0.6;
    Spheres[2][2] = 0.6;
    Spheres[2][3] = -2;
    Spheres[2][4] = 0.01;
    Spheres[2][5] = 1;
    Spheres[2][6] = 0.5;
    Spheres[2][7] = -1;
    Spheres[2][8] = -1;

    // Sphere 4 : Miroir parfait
    Spheres[3][0] = 0.3;
    Spheres[3][1] = -0.6;
    Spheres[3][2] = -0.6;
    Spheres[3][3] = -2;
    Spheres[3][4] = 0.01;
    Spheres[3][5] = 0.01;
    Spheres[3][6] = 0.01;
    Spheres[3][7] = 1;
    Spheres[3][8] = -1;

    // Ajouter les sphères de la grille ( on ajoute des sphères derriere la caméra de couleur aléatoire pour mettre en valeur les reflexions)
    for (int i = 0; i < taille_grid_spheres_secondaires; i++) {
        for (int j = 0; j < taille_grid_spheres_secondaires; j++) {

            double CGridSphere[3] = { ((-1)^j) * (j+1) * ecartementSpheres, ((-1)^i) * (i+1) * ecartementSpheres + 5, ecartementSpheres };
            
            double AGridSphere[3];
            double* AGridSphere_ptr=randomVecteur3(); // on génère une absorption random selon une loi normale.
            for (int k = 0; k < 3; k++) {
                AGridSphere[k] = *(AGridSphere_ptr+k);
            }

            int idxSphere = i * taille_grid_spheres_secondaires + j + nombre_de_spheres_principales;
            Spheres[idxSphere][0] = RGridSpheres;
            Spheres[idxSphere][1] = CGridSphere[0];
            Spheres[idxSphere][2] = CGridSphere[1];
            Spheres[idxSphere][3] = CGridSphere[2];
            Spheres[idxSphere][4] = AGridSphere[0];
            Spheres[idxSphere][5] = AGridSphere[1];
            Spheres[idxSphere][6] = AGridSphere[2];
            Spheres[idxSphere][7] = GGridSpheres;
            Spheres[idxSphere][8] = NGridSpheres;
        }
    }
}

#endif