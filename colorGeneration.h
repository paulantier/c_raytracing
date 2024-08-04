#ifndef COLORGENERATION_H
#define COLORGENERATION_H

#include <random>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "generationParameters.h"

#pragma once


std::default_random_engine generator(time(NULL));
std::normal_distribution<double> distribution(0.0,1.0);

double imgAbsorptionTextureSol[heightTextureSol][widthTextureSol][3];
double imgAbsorptionTexturePlanete[heightTexturePlanete][widthTexturePlanete][3];



double *randomVecteur3() { // on génère un vecteur aléatoire normalisé suivant une loi normale
    static double result[3]; // Tableau statique pour stocker les valeurs R, G, B
    double normeVecteurRandom;
    for (int i = 0; i < 3; i++) {
        double randomNormal = distribution(generator); // on utilise random de c++ (c'est la seule chose dans tout le projet qui n'est pas du c
        // par souci de simplicité)
        result[i] = randomNormal;
        normeVecteurRandom += randomNormal*randomNormal;
    }
    normeVecteurRandom=sqrt(normeVecteurRandom);
    
    for (int i = 0; i < 3; i++) {
        result[i]/=normeVecteurRandom;
    }

    return result;
}

// Cette fonction donne la valeur d'absorption rgb d'un rayon, à partir des coordonnées d'une intersection entre un rayon et un plan (d'équation y=constante)
// dans le repère monde et de l'image de la texture.
double* absorptionTexturePointIntersection(double pointInter[3], double coeffMapTexture, double absorptionTexturePixel[3]) {

// On suppose la texture appliquée sur un plan d'équation Y = constante donc
    int UpointInter2D = (int) coeffMapTexture*pointInter[0]; // On récupère la valeur en x et en z de notre point d'intersection
    int VpointInter2D = (int) coeffMapTexture*(-pointInter[2]); // ce qui correspond à des coordonnées de pixel (u,v) modulo la taille de l'image de texture.

    UpointInter2D = UpointInter2D % heightTextureSol; // on fait le modulo des coordonnées x et z de l'intersection pour avoir des coordonnées en pixels

    VpointInter2D = VpointInter2D % widthTextureSol;

    if (UpointInter2D<0) { // le modulo n en C est entre -n+1 et n donc on doit décaler si c'est dans les négatifs
        UpointInter2D+=heightTextureSol;
    }

    if (VpointInter2D<0) {
        VpointInter2D+=widthTextureSol;
    }

    for (int i = 0; i < 3; i++) {
        double val8bit=imgAbsorptionTextureSol[UpointInter2D][VpointInter2D][i]; // On prend dans l'image de texture le pixel correspondant
        absorptionTexturePixel[i]=1-val8bit/255;
    }

    return absorptionTexturePixel;
}

double* absorptionUVMapping(double pointInter[3], double absorptionTexturePixel[3]){

    double xSphere = (pointInter[0]-xCentrePlanete);
    double ySphere = (pointInter[1]-yCentrePlanete);
    double zSphere = (pointInter[2]-zCentrePlanete);
    double rayonSphere=sqrt(xSphere*xSphere+ySphere*ySphere+zSphere*zSphere); // On utilise pas le vrai rayon de la sphère car
    // les erreurs d'arrondis peuvent causer uTexture ou vTexture d'atteindre des valeurs hors des dimensions de l'image

    // Pour déterminer les coordonnées équivalentes dans l'image à partir du point d'intersection sur la sphère on utilise
    // les expressions de base du UV mapping d'une sphère.
    double uTexture = widthTexturePlanete*(0.5 + atan2f(zSphere/rayonSphere, -xSphere/rayonSphere) / (2 * M_PI));
    double vTexture = heightTexturePlanete*(0.5 + asinf(-ySphere/rayonSphere) / M_PI);

    int uImage = (int)vTexture;
    int vImage = (int)uTexture;

    for (int i = 0; i < 3; i++) {
        double val8bit=imgAbsorptionTexturePlanete[uImage][vImage][i]; // On associe le pixel correspondant.
        absorptionTexturePixel[i] = 1 - val8bit/255;
    }
    return absorptionTexturePixel;
}

#endif