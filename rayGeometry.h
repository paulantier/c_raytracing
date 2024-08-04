#ifndef RAYGEOMETRY_H
#define RAYGEOMETRY_H

// Pour ce header qui contient toutes les fonction de calcul géométriques on a besoin de cmath
#include <cmath>
// On a aussi besoin d'autres headers car c'est dans rayGeometry qu'on attribue la valeur
// de couleurs aux rayons à partir des images si on touche la planète ou le plan
#include "colorGeneration.h"

// On a aussi besoin des constantes de générations de l'image
#include "generationParameters.h"


// On crée plusieurs constantes de générations propres à ce fichier :
double dezoomTexture=10.0; // permet de choisir à quel point la texture du sol et zoomée
int lumiereConstante=1; // permet de décider si la lumière du ciel est constante peu importe la direction où si on impose un gradient d'intensité
double coeffAttenuationAngle=1; // lié à la variable lumiereConstante, décide de l'attenuation de lumiere en fonction de l'angle
double GSol=1; // coefficient de glossiness du sol.

// vecteur caractérisant le blanc dans le cas où un rayon ne rencontre plus d'objet (simule la lumière du ciel)
double finDeRayon[3] = {255, 255, 255};
// Absorption de référence, on lui soustrait l'absorption de chaque objet rencontré pour savoir quelle lumière rgb est renvoyée par l'objet
// à partir du rayon précédent
double Aref[3] = {1, 1, 1};

// valeurs d'Absorption, de centre de Sphère et de glossiness de la sphère intersectée en premier par le rayon (au sens de zBuffer)
double Amin[3];
double Cmin[3];
double Gmin;

// tInter est la distance entre le point d'intersection entre le rayon est un objet et le point d'émission du rayon
// il correspond au scalaire par lequel il faut multiplier notre vecteur directeur normalisé pour intersecter un objet
double tInter;
double tInterPlanete=1e20; // idem que tInter mais pour la planète, elle a un traitement spécial et donc il faut initialiser cette variable
// car on teste si tInter==tInterPlanete même si la planète n'est pas intersectée


// Ces objets servent à stocker les valeurs de sortie de chaque sous fonction pour les utiliser dans la fonction centrale de ce header qu'est lancerRayon()
double vectInterResult[9];
double intensiteRayonReflechi[3];
double intensiteRayonRefracte[3];
double intensiteRayon[3];
double intersectionData[7];

double reflexionPlan[3];
double normalePlan[3];

double reflexionFancyResult[3];
double refractionResult[4];

// cette fonction calcule si un rayon intersecte une sphère de centre et de rayon données et calcule à quel distance cela se produit.
double* interRaySphere(double Rsphere, double Csphere[3], double origineRayon[3], double directionRayon[3]) {

    static double result[2]; // Tableau statique pour stocker le résultat [boolIntersection, t]

    // Les coordonnées de l'intersection entre une droite et une sphère s'obtiennent en cherchant les racines d'un polynôme de dégré 2
    double a = directionRayon[0] * directionRayon[0] + directionRayon[1] * directionRayon[1] + directionRayon[2] * directionRayon[2];
    double R0moinsC[3] = {origineRayon[0] - Csphere[0], origineRayon[1] - Csphere[1], origineRayon[2] - Csphere[2]};
    double b = 2 * (directionRayon[0] * R0moinsC[0] + directionRayon[1] * R0moinsC[1] + directionRayon[2] * R0moinsC[2]);
    double c = R0moinsC[0] * R0moinsC[0] + R0moinsC[1] * R0moinsC[1] + R0moinsC[2] * R0moinsC[2] - Rsphere * Rsphere;
    // On calcule le discriminant
    double delta = b * b - 4.0 * a * c;

    // Si le discriminant est positif, deux intersections (ou une intersection si les deux sont confondues mais aucune différence pour nous);
    if (delta >= 0) {
        double sqrtDelta=sqrt(delta);
        double a2= 2* a;
        double t1 = (-b - sqrtDelta) / a2;
        double t2 = (-b + sqrtDelta) / a2;
        double t = t1 < t2 ? t1 : t2; // On prend le minimum entre t1 et t2, car c'est le premier point de la sphère intersecté

        // Si tInter est nul (ou quasi nul) on prend l'autre (car si on est à l'intérieur d'une sphère on veut que le rayon traverse la sphère et qu'il ne prennent pas
        // en compte l'intersection où il est déjà)
        if (abs(t) <= 0.0000001) {
            t = t1 > t2 ? t1 : t2;
        }

        // On met dans notre tableau qu'il y a bien une intersection, et le t associé
        result[0] = 1; // True
        result[1] = t;
    } 

    else { // Si le discriminant est négatif la droite n'intersecte pas la sphère.
        result[0] = 0; // False
        result[1] = 0;
    }

    return result;
}


// cette fonction calcule si un rayon intersecte un plan d'equation donnée et calcule à quel distance cela se produit.
double* interRayPlan(double eqPlan[4], double origineRayon[3], double directionRayon[3]) {
    
    // retourne 7 flottants
    // le premier est lambda qui correspond au multiple du directionRayon correspondant au point d'intersection
    // les 3 flottants suivants sont le vecteur normal au plan
    // les 3 flottants suivant sont les coordonnées du point d'intersection

    // On calcule le dénominateur de position d'intersection (cela revient à résoudre un système)
    double denominator = eqPlan[0] * directionRayon[0] + eqPlan[1] * directionRayon[1] + eqPlan[2] * directionRayon[2];

    if (denominator != 0)  // On vérifie que le plan est intersecté par la droite générée par le directionRayon à partir du point origineRayon
    {
        // lambda ici correspond au t d'intersection pour la sphère, c'est à dire le multiple du vecteur directeur du rayon pour lequel on intersecte.
        double lambda = (eqPlan[0] * origineRayon[0] + eqPlan[1] * origineRayon[1] + eqPlan[2] * origineRayon[2] + eqPlan[3] - eqPlan[4]) / denominator;
        
        if (lambda>0.0000001) { // On vérifie que lambda est positif et qu'on intersecte bien un objet dans le sens du rayon et pas dans la direction opposée au rayon
        // On évite aussi le cas où le vecteur réfléchi par le plan calcule qu'il intersecte le plan pour lambda = 0 car il est déjà réfléchi par le plan
            intersectionData[0] = lambda;

            // On calcule la normale au plan (trivial par définition de l'équation cartésienne de plan, on normalise au cas où)
            double normeNormale = sqrt(eqPlan[0] * eqPlan[0] + eqPlan[1] * eqPlan[1] + eqPlan[2] * eqPlan[2]);
            double normaleX = eqPlan[0] / normeNormale;
            double normaleY = eqPlan[1] / normeNormale;
            double normaleZ = eqPlan[2] / normeNormale;

            // Normale au plan (=vecteur directeur en soi)
            intersectionData[1] = normaleX;
            intersectionData[2] = normaleY;
            intersectionData[3] = normaleZ;

            //Position de l'intersection
            intersectionData[4] = origineRayon[0] + lambda * directionRayon[0];
            intersectionData[5] = origineRayon[1] + lambda * directionRayon[1];
            intersectionData[6] = origineRayon[2] + lambda * directionRayon[2];

            //Rayon réfléchi, le calcul sera détaillé dans le rapport.
            double dotProduct = 2 * (directionRayon[0] * normaleX + directionRayon[1] * normaleY + directionRayon[2] * normaleZ);
            intersectionData[7] = directionRayon[0] - dotProduct * normaleX;
            intersectionData[8] = directionRayon[1] - dotProduct * normaleY;
            intersectionData[9] = directionRayon[2] - dotProduct * normaleZ;

        }
        else { // si lambda est strictement supérieur à 0 on retourne une liste de -1
            for (int i = 0; i < 10; i++) {
                intersectionData[i] = -1;
            }
        }
    } 
    else // s'il n'y a pas d'intersection on retourne une liste de -1
    {
        for (int i = 0; i < 10; i++) {
        intersectionData[i] = -1;
        }
    }
    return intersectionData;
}


// cette fonction calcule la position, la normale et le vecteur réfléchi pour une intersection connue
double* vectInterSphere(double origineRayon[3], double directionRayon[3], double t, double Csphere[3]) {
    static double result[9]; // Tableau statique pour stocker les résultats [pointInter[3], reflexionInter[3], normaleInter[3]]

    // On calcule le point d'intersection par définition de t
    double pointInter[3];
    for (int i = 0; i < 3; i++) {
        pointInter[i] = origineRayon[i] + directionRayon[i] * t;
    }

    // On calcule la normale à l'intersection simplemenent en prenant le vecteur de même direction que le rayon de la sphère passant par l'intersection.
    double normaleInter[3];
    double distance = 0;
    for (int i = 0; i < 3; i++) {
        normaleInter[i] = pointInter[i] - Csphere[i];
        distance += normaleInter[i] * normaleInter[i]; // ici distance est la distance au carré, càd le carré du rayon normalement
        // mais on fait le calcul pour éviter les arrondis en cascade
    }

    distance = sqrt(distance);
    for (int i = 0; i < 3; i++) {
        normaleInter[i] /= distance;
    }


    // Ce calcul sera justifié dans le rapport mais on exprime le vecteur réfléchi en fonction du vecteur incident et de la normale.
    double dotProduct = 0;
    for (int i = 0; i < 3; i++) {
        dotProduct += directionRayon[i] * normaleInter[i];
    }

    double reflexionInter[3];
    for (int i = 0; i < 3; i++) {
        reflexionInter[i] = -2 * dotProduct * normaleInter[i] + directionRayon[i];
    }

    for (int i = 0; i < 3; i++) {
        result[i] = pointInter[i];
        result[i + 3] = reflexionInter[i];
        result[i + 6] = normaleInter[i];
    }

    return result;
}

// cette fonction modifie la direction du vecteur réfléchi après intersection en fonction
//  de si l'objet est mate (réflexion lambertienne) ou brillant (reflexion glossy)
double* reflexionFancy(double normaleInter[3], double reflexionInter[3], double Gsphere) {
    static double nvReflexion[3]; // Tableau statique pour stocker le résultat

    // Gsphere est l'indice de glossiness de la sphère (par extension de l'objet) intersectée.
    if (Gsphere <= 1 && Gsphere >= 0) { // Si la glossiness est positive (et définie correctement)

        double* vecteurRandomSphere_ptr = randomVecteur3(); // On crée un vecteur aléatoire dont les trois coordonnées sont générées par une gaussienne
        // qu'on normalise pour obtenir un vecteur unitaire aléatoire dans une sphère, ce qui sera justifié dans le rapport.
        double vecteurRandomSphere[3];
        for(int i = 0; i < 3; i++){
            vecteurRandomSphere[i] = *(vecteurRandomSphere_ptr + i);
        }

        for (int i = 0; i < 3; i++) { // On ajoute à notre vecteur réflechi un vecteur sphérique  qui aura beaucoup d'impact si la sphère et peu brillante
        // et aucun impact si la sphère est un miroir pur.
            nvReflexion[i] = reflexionInter[i] + (1 - Gsphere) * vecteurRandomSphere[i]; 
        }
    } 
    
    else if (Gsphere == -1) { // On pose la convention que si la glossiness vaut -1 on a un objet mate et on applique une réfléxion lambertienne
        
        double* vecteurRandomSphere_ptr = randomVecteur3(); // on génère le même vecteur sphérique aléatoire
        double vecteurRandomSphere[3];
        for(int i = 0; i < 3; i++){
            vecteurRandomSphere[i] = *(vecteurRandomSphere_ptr + i);
        }

        for (int i = 0; i < 3; i++) {
            nvReflexion[i] = normaleInter[i] + vecteurRandomSphere[i]; // On l'ajoute cette fois ci à notre vecteur normal.
        }

    } 
    
    else { // Si on a aucune indication de réflexion alors on garde le vecteur réfléchi tel quel.
        for (int i = 0; i < 3; i++) {
            nvReflexion[i] = reflexionInter[i];
        }
    }
    
    // quelque soit l'opération on normalise ensuite notre vecteur pour s'assurer d'avoir tout le temps des vecteurs unitaires en sortie de fonction
    double normeNvReflexion=sqrt(nvReflexion[0]*nvReflexion[0]+nvReflexion[1]*nvReflexion[1]+nvReflexion[2]*nvReflexion[2]);
    for (int i = 0; i < 3; i++) {
        nvReflexion[i] = nvReflexion[i]/normeNvReflexion;
    }

    return nvReflexion;
}

// cette fonction calcule la direction de la réfraction du rayon par l'objet en fonction d'indice de milieu
// du rayon incidente et de l'indice du milieu de l'objet
double* refraction(double normaleInter[3], double incidentInter[3], double N1, double N2) {
    // Sortie : Tableau pour stocker le résultat [coeffReflexion, refracteInter[0], refracteInter[1], refracteInter[2]]

    double refracteInter[3];
    double coeffReflexion; // cette valeur indique quel proportion de l'intensité lumineuse sera réfléchie et laquelle sera réfractée
    // ce coefficient dépend du rayon incident et des différents milieux.
    
    if (N2 < 1) { // si l'indice de vitesse de la lumière dans le milieu est incorrecte on renvoie le vecteur tel quel.
        for (int i = 0; i < 3; i++) {
            refracteInter[i] = incidentInter[i];
            // (il ne sera pas utilisé grâce au coeff de reflexion mis à 1)
        }
        coeffReflexion = 1;
    }

    else { // si tout va bien :
        double nRatio = N1 / N2; // on appelle nRatio le rapport des deux indices ce qui va faciliter les calculs.
        double cosIncident = normaleInter[0] * incidentInter[0] + normaleInter[1] * incidentInter[1] + normaleInter[2] * incidentInter[2]; 
        // On calcule le cosinus du vecteur incident avec un produit scalaire.

        // On calcule le carré du sinus réfracté avec les lois de Snell-Descartes (ou plutôt de Ibn Sahl) et de la trigonométrie.
        double sinRefracteSqr = nRatio * nRatio * (1 - cosIncident * cosIncident);

        // Si le carré du sinus est supérieur à 1, c'est incohérent, et cela correspond à la situation de réflexion totale, pas de réfraction
        if (sinRefracteSqr > 1) {
            for (int i = 0; i < 3; i++) {
                refracteInter[i] = incidentInter[i]; // on renvoie le vecteur d'entrée dans ce cas
                // (il ne sera pas utilisé grâce au coeff de reflexion mis à 1)
            }
            coeffReflexion = 1;
        }

        else { // Si il y a réfraction :
            double cosRefracte = sqrt(1 - sinRefracteSqr); // on calcule les éléments nécessaires à la détermination du rayon réfracté : cf rapport

            double normeRefraction=0;

            for (int i = 0; i < 3; i++) {
                refracteInter[i] = nRatio * incidentInter[i] + (nRatio * cosIncident - cosRefracte) * normaleInter[i];
                normeRefraction+=refracteInter[i]*refracteInter[i];
            }

            normeRefraction=sqrt(normeRefraction);

            for (int i = 0; i < 3; i++){
                refracteInter[i]/=normeRefraction;
            }

            double N2cos1 = N2 * abs(cosIncident);
            double N1cos2 = N1 * abs(cosRefracte); 

            coeffReflexion = abs(N2cos1-N1cos2) / (N2cos1+N1cos2);

        }

    }
    // On renvoie le coefficient de réflexion et un vecteur analogue au vecteur réfracté calculé (on prend l'opposé selon x et y car cela fonctionne)
    // mais je ne comprend pas pourquoi, j'ai trouvé ceci en analysant mes résultats d'image.
    refractionResult[0] = coeffReflexion;
    refractionResult[1] = -refracteInter[0];
    refractionResult[2] = -refracteInter[1];
    refractionResult[3] = refracteInter[2];

    return refractionResult;
}

// cette fonction est celle qui réunit toutes les précédentes pour calculer récursivement la valeur RGB d'un pixel de notre image en fonction de nos
// objets. Cette fonction est conséquente et contient de nombreuses exceptions car les objets sphère, sol et planète sont différents et doivent appeler
// des techniques et des calculs différents. Cette fonction est récursive et la limite de profondeur de l'arbre de récursive est donnée par la
// variable rayons_restants.
double* lancerRayon(double origineRayon[3], double directionRayon[3], int rayons_restants, double Nmilieu, double Spheres[nombre_de_spheres][9], double Planete[9], double eqPlan[4]) {
    
    static double intensiteRayonIncident[3]; // Tableau statique pour stocker l'intensité du rayon incident
    double Nmin; // indice de milieu de la première sphère intersectée
    if (rayons_restants < 1) { // On réduit le nombre de rayons restants max de 1
        for (int i = 0; i < 3; i++) {
            intensiteRayonIncident[i] = finDeRayon[i];
        }
        return intensiteRayonIncident;
    }

    // on définit des variables utiles
    double tMin = 1e10; // la distance de la plus petite intersection
    int aucuneInter = 1; // le booléen qui vérifie s'il y a une intersection
    int notASphere=0; // le booléen qui vérifie s'il y a une intersection avec un objet qui n'est pas une sphère
    double Rsphere; // le rayon de la sphère
    double Csphere[3]; // le centre de la sphère
    double Asphere[3]; // l'absorption rgb de la sphère
    double Gsphere; // le coefficient de glossiness de la sphère
    double Nsphere; // l'indice de milieu de la sphère
    double intersectionPlan[3]; // les coordonnées d'intersection entre le rayon et le plan
    double absorptionTexturePixel[3]; // l'absorption rgb d'un pixel à partir des données d'une image de texture


    for (int idxSphere = 0; idxSphere < nombre_de_spheres; idxSphere++) { // on teste les intersections avec toutes les sphères
        if (idxSphere==nombre_de_spheres-1){ // la dernière sphère est la planète
            Rsphere = Planete[0];
            for (int j = 0; j < 3; j++) {
                Csphere[j]=Planete[j+1];
                Asphere[j]=Planete[j+4];
            }
            Gsphere = Planete[7];
            Nsphere = Planete[8];
        }
        else{
            Rsphere = Spheres[idxSphere][0];
            for (int j = 0; j < 3; j++) {
                Csphere[j]=Spheres[idxSphere][j+1];
                Asphere[j]=Spheres[idxSphere][j+4];

            }
            Gsphere = Spheres[idxSphere][7];
            Nsphere = Spheres[idxSphere][8];
        }
        

        double* interResult = interRaySphere(Rsphere, Csphere, origineRayon, directionRayon); // On calcule l'intersection entre notre rayon et la sphère

        int boolInter = (int)interResult[0];
        tInter = interResult[1]; // On récupère le booléen qui vérifie si le rayon intersecte la sphère

        if (boolInter && tInter > 0 && tInter < tMin) {
            aucuneInter = 0;
            tMin = tInter;
            if (idxSphere==nombre_de_spheres-1){
                tInterPlanete=tInter; // Dans le cas où la sphère en question est la planète
            }
            
            // On récupère les valeurs de la sphère intersectée la plus proche
            Cmin[0] = Csphere[0];
            Cmin[1] = Csphere[1];
            Cmin[2] = Csphere[2];

            Amin[0] = Asphere[0];
            Amin[1] = Asphere[1];
            Amin[2] = Asphere[2];

            Gmin = Gsphere;
            Nmin = Nsphere;
        }
    }

    
    // On vérifie de même si l'intersection entre le rayon et le plan est la première intersection.
    double* interResultPlan = interRayPlan(eqPlan, origineRayon, directionRayon);

    tInter = interResultPlan[0];

    if (tInter > 0 && tInter<tMin) {
        aucuneInter = 0;
        tMin = tInter;
        notASphere=1; // on change la valeur du booléen notASphere qui fera les calculs pour les plans et non pour la sphère pour les vecteurs réflechi/réfracté

        for (int i = 0; i < 3; ++i) {
            normalePlan[i]=*(interResultPlan + i + 1);
            intersectionPlan[i]=*(interResultPlan + i + 4);
            reflexionPlan[i]=*(interResultPlan + i +7);
        }
    }


    if (aucuneInter) {
        if (lumiereConstante==1){ // cette boucle est un essai pour appliquer un gradient de luminosité de la lumière du ciel en fonction
        // de l'angle par rapport à la verticale de l'espace
            for (int i = 0; i < 3; i++) {
                intensiteRayonIncident[i] = finDeRayon[i];
            }
        }
        else {
            coeffAttenuationAngle = (directionRayon[2] > 0) ? directionRayon[2] : 0;
            for (int i = 0; i < 3; i++) {
                intensiteRayonIncident[i] = (1-coeffAttenuationAngle)*finDeRayon[i];
            }
        }
        return intensiteRayonIncident;
    }
    
    else if (notASphere) { // Dans le cas de la réflexion pour un plan.
        
        rayons_restants--;
        
        /*
        //Cette section pose problème pour appliquer des réflexions lambertiennes et glossy dans le cas du rayon réfléchi par un plan donc
        //j'ai préféré l'enlever pour l'instant.

        double* reflexionFancyResult_ptr = reflexionFancy(normalePlan, reflexionPlan, GSol);
        for (int i = 0; i < 3; i++) {
            reflexionFancyResult[i] = *(reflexionFancyResult_ptr + i);
        }
        
        double* intensiteRayonReflechi_ptr = lancerRayon(intersectionPlan, reflexionFancyResult, rayons_restants, Nmilieu, Spheres, Planete, eqPlan);
        for (int i = 0; i < 3; i++) {
            intensiteRayonReflechi[i] = *(intensiteRayonReflechi_ptr + i);
        }

        double* absorptionTexturePointIntersection_ptr = absorptionTexturePointIntersection(intersectionPlan, dezoomTexture, absorptionTexturePixel);
        for (int i = 0; i < 3; i++) {
            absorptionTexturePixel[i] = *(absorptionTexturePointIntersection_ptr+ i);
        }

        for (int i = 0; i < 3; i++) {
            intensiteRayonIncident[i] = (Aref[i] - absorptionTexturePixel[i]) * intensiteRayonReflechi[i];
        }
        */
        

       // ainsi pour déterminer la couleur du sol on regarde juste la valeur du pixel de l'image de texture correspondant
        double* absorptionTexturePointIntersection_ptr = absorptionTexturePointIntersection(intersectionPlan, dezoomTexture, absorptionTexturePixel);
        for (int i = 0; i < 3; i++) {
            intensiteRayonIncident[i] = (1-*(absorptionTexturePointIntersection_ptr+ i)) * 255;
        }

        return intensiteRayonIncident; 

    }

    else { // dans le cas d'une réflexion par une sphère classique
        rayons_restants--;
        // On calcule la position la normale et la réflexion de l'intersection
        double* vectInterResult_ptr = vectInterSphere(origineRayon, directionRayon, tMin, Cmin);
        for (int i = 0; i < 9; i++) {
            vectInterResult[i] = *(vectInterResult_ptr + i);
        }

        if (tMin==tInterPlanete) { // si on est dans le cas de la planète
            double* Aminptr=absorptionUVMapping(vectInterResult, absorptionTexturePixel); //pour déterminer la valeur de l'absorption pour
            // ce rayon on utilise une UV map de la Terre et on associe le pixel correspondant dans le planisphère.
            for (int i=0; i<3; i++){
                Amin[i]=*(Aminptr + i);
            }
        }

        double refractionResult[4];
        if (Nmin==Nmilieu){ // Dans le cas où la sphère intersectée la plus proche a le même indice de refraction
        // que l'environnement dans lequel on a lancé le rayon, càd quand on est dans une sphère après réfraction
        // et qu'on doit sortir de la sphère.

            double normaleOpposee[3]; // On prend l'opposé de la normale car elle était calculée en normalisant pointIntersection-centreSphere et donc
            // ici la normale est à l'opposée.
            for(int i = 0; i < 4; i++){
                normaleOpposee[i]=-vectInterResult[i+6];
            }

            double* refractionResult_ptr = refraction(normaleOpposee, directionRayon, Nmin, Nespace);
            for (int i = 0; i < 4; i++) {
                refractionResult[i] = *(refractionResult_ptr + i);

            }
        }
        else{ // on calcule la réfraction entrante
            double* refractionResult_ptr = refraction(vectInterResult + 6, directionRayon, Nmilieu, Nmin); 
            for (int i = 0; i < 4; i++) {
                refractionResult[i] = *(refractionResult_ptr + i);
            }
        }
        // dans tous les cas on calcule la reflexion pour la sphère
        double* reflexionFancyResult_ptr = reflexionFancy(vectInterResult + 6, vectInterResult + 3, Gmin);
        for (int i = 0; i < 3; i++) {
            reflexionFancyResult[i] = *(reflexionFancyResult_ptr + i);
        }

        if (refractionResult[0] >= 1) { //si le coefficient de réflexion est égal à 1 (pour réduire le nombre de calculs)
            double* intensiteRayonReflechi_ptr = lancerRayon(vectInterResult, reflexionFancyResult, rayons_restants, Nmilieu, Spheres, Planete, eqPlan);
            for (int i = 0; i < 3; i++) {
                intensiteRayonReflechi[i] = *(intensiteRayonReflechi_ptr + i);
            }
            
            for (int i = 0; i < 3; i++) {
                intensiteRayonIncident[i] = (Aref[i] - Amin[i]) * intensiteRayonReflechi[i];
            }
        }
        else if (refractionResult[0] <= 0) { //si le coefficient de réflexion est égal à 0 (idem)
            double* intensiteRayonRefracte_ptr = lancerRayon(vectInterResult, refractionResult+1, rayons_restants, Nmin, Spheres, Planete, eqPlan);
            for (int i = 0; i < 3; i++) {
                intensiteRayonRefracte[i] = *(intensiteRayonRefracte_ptr + i);
            }
            
            for (int i = 0; i < 3; i++) {
                intensiteRayonIncident[i] = (Aref[i] - Amin[i]) * intensiteRayonRefracte[i];
            }
        }
        else {
            if (Nmin==Nmilieu){ //boucle de refraction sortant de la sphère : on utilise la normale inversée.
            // On lance deux rayons pour avoir la réflexion et la réfraction ce qui augmente grandement la taille de l'arbre de récursion
                double* intensiteRayonRefracte_ptr = lancerRayon(vectInterResult, refractionResult+1, rayons_restants, Nespace, Spheres, Planete, eqPlan);
                for (int i = 0; i < 3; i++) {
                    intensiteRayonRefracte[i] = *(intensiteRayonRefracte_ptr + i);
                }
                double* intensiteRayonReflechi_ptr = lancerRayon(vectInterResult, reflexionFancyResult, rayons_restants, Nmilieu, Spheres, Planete, eqPlan);
                for (int i = 0; i < 3; i++) {
                    intensiteRayonReflechi[i] = *(intensiteRayonReflechi_ptr + i);
                }
                for (int i = 0; i < 3; i++) {
                    // On prend en compte les deux rayons proportionnellement au coefficient de réfraction
                    intensiteRayon[i] = (refractionResult[0] * intensiteRayonReflechi[i] + (1 - refractionResult[0]) * intensiteRayonRefracte[i]);
                    intensiteRayonIncident[i] = (Aref[i] - Amin[i]) * intensiteRayon[i];
                }
            }
            else{ // boucle de refraction entrant dans la sphère 
                double* intensiteRayonRefracte_ptr = lancerRayon(vectInterResult, refractionResult+1, rayons_restants, Nmin, Spheres, Planete, eqPlan);
                for (int i = 0; i < 3; i++) {
                    intensiteRayonRefracte[i] = *(intensiteRayonRefracte_ptr + i);
                }
                double* intensiteRayonReflechi_ptr = lancerRayon(vectInterResult, reflexionFancyResult, rayons_restants, Nmilieu, Spheres, Planete, eqPlan);
                for (int i = 0; i < 3; i++) {
                    intensiteRayonReflechi[i] = *(intensiteRayonReflechi_ptr + i);
                }
                for (int i = 0; i < 3; i++) {
                    intensiteRayon[i] = (refractionResult[0] * intensiteRayonReflechi[i] + (1 - refractionResult[0]) * intensiteRayonRefracte[i]);
                    intensiteRayonIncident[i] = (Aref[i] - Amin[i]) * intensiteRayon[i];
                }
            }
        }

        return intensiteRayonIncident;
    }
}


#endif