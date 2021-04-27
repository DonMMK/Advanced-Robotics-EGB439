
close all; clear; clc
load queensland_towns
[p,f,e]=graph_planner(distanceMatrix, placeCoords, placeNames, 5, 1)