# To add a new cell, type '# %%'
# To add a new markdown cell, type '# %% [markdown]'
# %%
from IPython import get_ipython
import pandas as pd


# %%
from math import *
import math
import statistics
from collections import OrderedDict 
import itertools
import time
import glob
import os
import csv
from os.path import dirname, join
ROOT_DIR = dirname(dirname(__file__))

import matplotlib as plt

#needed to model temps through year from monthly averages
from scipy.interpolate import *

import pandas as pd
import numpy as np

#eee
#Setting constants for code with classes

SOLAR_CONSTANT = 1360. #W*m^-2
tau = 0.7 #0.7 clear day
STEFAN_BOLTZMANN = 5.670373*10**(-8) #W*m^-2*K^-4
ALBEDO = 0.4 #ground reflectance (albedo) of dry sandy soil ranges 0.25-0.45
#E_G = 0.9 ##surface emissivity of sandy soil w/<2% organic matter approx 0.88
OMEGA = math.pi/12.
u = 0.1 #windspeed in m/s

cp=29.3 #unit = J/(mol*C) #specific heat of air at constant temp
Tb1=[35]
e=0.01675 #earth essentricity 
A_S = 0.8 #shortwave rad absorbance of animal
A_L = 0.95 #longwave rad absorbance of animal
s = 1.0 #proportion of animal in sun

scenarios= ['undulatus_utah','occipitalis_ecuador','undulatus_AZ','clarki_AZ','ornatus_AZ','graciosus_kolob','graciosus_mtdiablo','scitulus_NM','mucronatus_usa','grammicus_mexico','grammicus_laguna','grammicus_paredon','maculata_nebraska','undulatus_nebraska','undulatus_newjersey','merriami_usa','boskianus_gabal','boskianus_mallahat','jarrovi_AZ','virgatus_AZ','mamorata_durango','stanburiana_durango','podacris_hispanica_salamanca','psammodromus_algirus_salamanca','psammodromus_hispanicus_salamanca','schreiberi_salamanca','lepida_ciudadrealspain','cantabrica_asturias','vivipara_asturias','cyreni_avila','hispanica_asturias','muralis_asturias','atrata_columbretes','erythrurus_madrid','tachydromoides_honsu','vivipara_antwerpen','agilis_limberg','viridis_loire','draconoides_maricopa','ornatus_maricopa','cornutum_cochise','maculata_cochise','modestum_cochise','ornatus_cochise','texanus_cochise','undulatus_cochise','wislizeni_nevada','undulatus_yavapai','ornatus_yuma','pectinata_morelos','monticola_mandeo','hispanicus_highland_central_mexico','hispanicus_lowland_central_mexico','undulatus_southcarolina','undulatus_texas','undulatus_ohio','undulatus_colorado','undulatus_consobrinus_newmexico','undulatus_tristichus_newmexico','undulatus_kansas','undulatus_garmani_nebraska','undulatus_georgia','nanuzae_brazil','rubrigularis_queensland','przewalskii_Alax_Zuoqi_China','przewalskii_Alax2_China','przewalskii_shandan', 'ornatus_grapevine','ornatus_animas','ornatus_chiricahua','ornatus_sunflower','lilfordi_mallorca','septentrionalis_chuzhou','septentrionalis_hangzhou','septentrionalis_lishui','septentrionalis_ningde','septentrionalis_guiyang','asio_mexico','braconnieri_mexico','cornutum_texas','coronatum_baja','douglassi_utah','mcalli_CA','modestum_NM_TX','orbiculare_mexico','platyrhinos_nevada','platyrhinos_utah','platyrhinos_mojave','platyrhinos_utah2','solare_arizona','maccoyi_canberra','guichenoti_canberra','coventryi_canberra','extrecasteauxii_canberra','tympanum_canberra','vittatus_veracruz','okadae_japan','undulatus_nebraska2','undulatus_newjersey2']


class Individual():
    def __init__(self,ectotherm_type,lizard_species,lizard_location,scenario,latitude,longitude,altitude,mass,length,width,emissivity,tpref_mean):
        'the classes object that holds the relevant morphological, physiological, and geographic information for the animal'
        self.type = ectotherm_type #lizard only
        self.lizard_spp = lizard_species
        self.lizard_location = lizard_location #city, state, country
        self.scenario = scenario #to split up different species and pops
        # self.corresponding_data = None 
        # mapping that defines the scenario to the correct CSV file
        self.scenarioLink= { # dictionary holding scenario microclimate data pairs #scenario Link could be any word
            "undulatus_utah":"microclimate/undulatus_utah.csv",
            "occipitalis_ecuador": "microclimate/occipitalis_ecuador.csv",
            "undulatus_AZ": "microclimate/undulatus_AZ.csv",
            "clarki_AZ": "microclimate/clarki_AZ.csv",
            "ornatus_AZ": "microclimate/ornatus_AZ.csv",
            "graciosus_kolob": "microclimate/graciosus_kolob.csv",
            "graciosus_mtdiablo": "microclimate/graciosus_mtdiablo.csv",
            "scitulus_NM": "microclimate/scitulus_NM.csv",
            "mucronatus_usa":"microclimate/mucronatus_usa.csv",
            "grammicus_mexico": "microclimate/grammicus_mexico.csv",
            "grammicus_laguna": "microclimate/grammicus_laguna.csv",
            "grammicus_paredon":"microclimate/grammicus_paredon.csv",
            "maculata_nebraska":"microclimate/maculata_nebraska.csv",
            "undulatus_nebraska":"microclimate/undulatus_nebraska.csv",
            "undulatus_newjersey":"microclimate/undulatus_newjersey.csv",
            "merriami_usa":"microclimate/merriami_usa.csv",
            "boskianus_gabal": "microclimate/boskianus_gabal.csv",
            "boskianus_mallahat":"microclimate/boskianus_mallahat.csv",
            "jarrovi_AZ":"microclimate/jarrovi_AZ.csv",
            "virgatus_AZ":"microclimate/virgatus_AZ.csv",
            "mamorata_durango":"microclimate/mamorata_durango.csv",
            "stanburiana_durango":"microclimate/stanburiana_durango.csv",
            "podacris_hispanica_salamanca":"microclimate/podacris_hispanica_salamanca.csv",
            "psammodromus_algirus_salamanca":"microclimate/psammodromus_algirus_salamanca.csv",
            "psammodromus_hispanicus_salamanca":"microclimate/psammodromus_hispanicus_salamanca.csv",
            "schreiberi_salamanca":"microclimate/schreiberi_salamanca.csv",
            "lepida_ciudadrealspain":"microclimate/lepida_ciudadrealspain.csv",
            "cantabrica_asturias":"microclimate/cantabrica_asturias.csv",
            "vivipara_asturias":"microclimate/vivipara_asturias.csv",
            "cyreni_avila":"microclimate/cyreni_avila.csv",
            "hispanica_asturias":"microclimate/hispanica_asturias.csv",
            "muralis_asturias":"microclimate/muralis_asturias.csv",
            "atrata_columbretes":"microclimate/atrata_columbretes.csv",
            "erythrurus_madrid":"microclimate/erythrurus_madrid.csv",
            # "septentrionalis_xinshau":"microclimate/septentrionalis_xinshau.csv",
            "tachydromoides_honsu":"microclimate/tachydromoides_honsu.csv",
            "vivipara_antwerpen":"microclimate/vivipara_antwerpen.csv",
            "agilis_limberg":"microclimate/agilis_limberg.csv",
            "viridis_loire":"microclimate/viridis_loire.csv",
            "draconoides_maricopa":"microclimate/draconoides_maricopa.csv",
            "ornatus_maricopa":"microclimate/ornatus_maricopa.csv",
            "cornutum_cochise":"microclimate/cornutum_cochise.csv",
            "maculata_cochise":"microclimate/maculata_cochise.csv",
            "modestum_cochise":"microclimate/modestum_cochise.csv",
            "ornatus_cochise":"microclimate/ornatus_cochise.csv",
            "texanus_cochise":"microclimate/texanus_cochise.csv",
            "undulatus_cochise":"microclimate/undulatus_cochise.csv",
            "wislizeni_nevada":"microclimate/wislizeni_nevada.csv",
            "undulatus_yavapai":"microclimate/undulatus_yavapai.csv",
            "ornatus_yuma":"microclimate/ornatus_yuma.csv",
            "pectinata_morelos":"microclimate/pectinata_morelos.csv",
            "monticola_mandeo":"microclimate/monticola_mandeo.csv",
            "hispanicus_highland_central_mexico":"microclimate/hispanicus_highland_central_mexico.csv",
            "hispanicus_lowland_central_mexico":"microclimate/hispanicus_lowland_central_mexico.csv",
            "undulatus_southcarolina":"microclimate/undulatus_southcarolina.csv",
            "undulatus_texas":"microclimate/undulatus_texas.csv",
            "undulatus_ohio":"microclimate/undulatus_ohio.csv",
            "undulatus_colorado":"microclimate/undulatus_colorado.csv",
            "undulatus_consobrinus_newmexico":"microclimate/undulatus_consobrinus_newmexico.csv",
            "undulatus_tristichus_newmexico":"microclimate/undulatus_tristichus_newmexico.csv",
            "undulatus_kansas":"microclimate/undulatus_kansas.csv",
            "undulatus_garmani_nebraska":"microclimate/undulatus_garmani_nebraska.csv",
            "undulatus_georgia":"microclimate/undulatus_georgia.csv",
            "nanuzae_brazil":"microclimate/nanuzae_brazil.csv",
            "rubrigularis_queensland":"microclimate/rubrigularis_queensland.csv",
            "przewalskii_Alax_Zuoqi_China":"microclimate/przewalskii_Alax_Zuoqi_China.csv",
            "przewalskii_Alax2_China":"microclimate/przewalskii_Alax2_China.csv",
            "przewalskii_shandan":"microclimate/przewalskii_shandan.csv",

            "ornatus_grapevine":"microclimate/ornatus_grapevine.csv",
            "ornatus_animas":"microclimate/ornatus_animas.csv",
            "ornatus_chiricahua":"microclimate/ornatus_chiricahua.csv",
            "ornatus_sunflower":"microclimate/ornatus_sunflower.csv",
            "lilfordi_mallorca":"microclimate/lilfordi_mallorca.csv",
            "septentrionalis_chuzhou":"microclimate/septentrionalis_chuzhou.csv",
            "septentrionalis_hangzhou":"microclimate/septentrionalis_hangzhou.csv",
            "septentrionalis_lishui":"microclimate/septentrionalis_lishui.csv",
            "septentrionalis_ningde":"microclimate/septentrionalis_ningde.csv",
            "septentrionalis_guiyang":"microclimate/septentrionalis_guiyang.csv",
            "asio_mexico":"microclimate/asio_mexico.csv",
            "braconnieri_mexico":"microclimate/braconnieri_mexico.csv",
            "cornutum_texas":"microclimate/cornutum_texas.csv",
            "coronatum_baja":"microclimate/coronatum_baja.csv",
            "douglassi_utah":"microclimate/douglassi_utah.csv",
            "mcalli_CA":"microclimate/mcalli_CA.csv",
            "modestum_NM_TX":"microclimate/modestum_NM_TX.csv",
            "orbiculare_mexico":"microclimate/orbiculare_mexico.csv",
            "platyrhinos_nevada":"microclimate/platyrhinos_nevada.csv",
            "platyrhinos_utah":"microclimate/platyrhinos_utah.csv",
            "platyrhinos_mojave":"microclimate/platyrhinos_mojave.csv",
            "platyrhinos_utah2":"microclimate/platyrhinos_utah2.csv",
            "solare_arizona":"microclimate/solare_arizona.csv",
            "maccoyi_canberra":"microclimate/maccoyi_canberra.csv",
            "guichenoti_canberra":"microclimate/guichenoti_canberra.csv",
            "coventryi_canberra":"microclimate/coventryi_canberra.csv",
            "extrecasteauxii_canberra":"microclimate/extrecasteauxii_canberra.csv",
            "tympanum_canberra":"microclimate/tympanum_canberra.csv",
            "vittatus_veracruz":"microclimate/vittatus_veracruz.csv",
            "okadae_japan":"microclimate/okadae_japan.csv",
            "undulatus_nebraska2":"microclimate/undulatus_nebraska2.csv",
            "undulatus_newjersey2":"microclimate/undulatus_newjersey2.csv",
        }


        self.latitude = latitude #decimal degrees
        self.longitude = longitude #decimal degrees
        self.altitude = altitude #meters
        self.MASS = mass #grams
        self.H = length #SVL in m, variable h in original Sears code
        self.D = width #animal diameter in m, variable d in original Sears code
        #self.S = 1.0 + shade #proportion of animal exposed to direct solar radiation (non-shaded)
        #self.A_S = shortwave_absorbance #absorbance of organism to shortwave radiation
        #self.A_L = longwave_absorbance #absorbance of organism to longwave radiation
        self.E_G = emissivity #ground emissivity
        self.E_S = 0.97 #emissivity of organism
        self.tpref_mean = tpref_mean
        #self.Tb_shade = Tb_shade
        #self.Tb_sun = Tb_sun
        #self.Tpref_min = Tpref_min
        #self.Tpref_max = Tpref_max
        #self.active_hours = annual_activity_hours
    
    def read_relative_path(self, filename):
        return join(dirname(dirname(__file__)), filename)
    
    def orbit_correction(self,julian): #solar_rad function in sears code
        'orbital correction for day'
        return (1.+2.*e*math.cos(((2*math.pi)/365.)*julian))
    
    def direct_solar_radiation(self,julian): #solar_rad function in original code combines these two into one function
        'required for solar radiation'
        return self.orbit_correction(julian)*SOLAR_CONSTANT
        
    def f(self,julian): #f_var
        'requried for solar declination and equation of time'
        return 279.575 + (0.9856 * julian)
    
    def ET(self,julian):
        'equation of time'
        _f = radians(self.f(julian))
        return (-104.7*math.sin(_f)+596.2*math.sin(2*_f)+4.3*math.sin(3*_f)-12.7*math.sin(4*_f)-429.3*math.cos(_f)-2.0*math.cos(2*_f)+19.3*math.cos(3*_f))/3600.
    
    def LC(self,lon):
        'longitude correction'
        return ((lon%15)*4.0)/60

    def t0(self,lc,et):
        'solar noon'
        t = 12 + lc - et
        return t

    def hour(self,t,t_zero): #hour function in sears code, degrees of rotation needed to rotate the earth to specific number of hours
        'conversion for zenith angle'
        h = 15*(t-t_zero)
        return h

    def declin(self,julian): #declination angle, angle between equator and the suns rays. Depends only on time of year, not time of day.
        'declination angle'
        return degrees(math.asin(0.39785* math.sin(radians(278.97 + 0.9856 * julian + 1.9165 * math.sin(radians(356.6 + 0.9856 * julian))))))
        
    def zenith(self,julian,t): #Lambert's law says intensity of radiation is proportional to the cos of the angle between the suns rays and the normal to the irridiated surface
        'zenith angle'
        if math.acos(math.sin(radians(self.latitude))*math.sin(radians(self.declin(julian))) + math.cos(radians(self.latitude))*math.cos(radians(self.declin(julian)))*math.cos(radians(self.hour(t,(self.t0(self.LC(self.longitude),self.ET(julian))))))) >= 0.:
            return math.acos(math.sin(radians(self.latitude))*math.sin(radians(self.declin(julian))) + math.cos(radians(self.latitude))*math.cos(radians(self.declin(julian)))*math.cos(radians(self.hour(t,(self.t0(self.LC(self.longitude),self.ET(julian)))))))
        return 0.
      
    def m(self,julian,hrs):
        'optical air mass number'
        p_a = 101.3*math.exp(-self.altitude/8200) #p_a = air pressure
        if math.cos(self.zenith(julian,hrs))>=0.:
            return p_a/(101.3*(math.cos(self.zenith(julian,hrs))))
        return 0.
            
    def hS0(self,julian,hrs): #solar rad adjusted by cos of zenith angle
        'direct solar radiation'
        z = self.zenith(julian,hrs)
        if math.cos(z)>= 0.:
            return self.direct_solar_radiation(julian)*(math.cos(z))
        return 0.
            
    def hS(self,julian, hrs, tau): #correcting for radiation hitting the atmosphere to get DIRECT incident solar rad, hS
        'solar radiation, corrected'
        return self.hS0(julian,hrs)*tau**self.m(julian,hrs)

    def diffuse_solar(self,julian,hrs,tau): #Sd, diffuse shortwave, solar rad scattered from reflecting off clouds
        'diffuse solar radiation'
        return self.hS0(julian,hrs)*0.3*(1.-(tau**self.m(julian,hrs)))

    def reflected_radiation(self,julian,t,tau): #Sr, reflected shortwave rad
        'reflected solar radiation'
        return ALBEDO*self.hS(julian,t,tau)
               
    def view_factor(self,julian,t):
        'view factor, Fh'
        return (1.+((4.*(self.H)*math.sin(radians(degrees(self.zenith(julian,t)))))/(math.pi*(self.D))))/(4.+(4.*(self.H)/(self.D)))

    def longwave_sky(self, Ta): #Sla, only varies with Ta
        'longwave radiation from sky'
        return 53.1*10**-14*(Ta +273.15)**6.

    def longwave_ground(self,Tg):
        'longwave radiation from the ground'
        return self.E_G*STEFAN_BOLTZMANN*(Tg +273.15)**4.
      
    def dynamic_frame_load(self):
        # read in the CSV
        # concatenate the directory with the file name using join function
        # Join and adding strings with "+" operator are same
        return pd.read_csv(self.read_relative_path(self.scenarioLink[self.scenario]))
    
    def radiation_abs(self, julian, hour, s, A_S, tau, Ta, Tg, A_L):
        'total absorbed radiation'
        dynamic_data_frame = pd.read_csv(self.read_relative_path(self.scenarioLink[self.scenario]))
        if math.cos(self.zenith(julian,hour)) > 0.:
            return (s*A_S*(self.view_factor(julian,hour) * (self.hS(julian, hour, tau)) + (0.5*self.diffuse_solar(julian, hour, tau)) + (0.5*self.reflected_radiation(julian, hour, tau)))) + (0.5*A_L*(self.longwave_sky(Ta)+ self.longwave_ground(Tg)))
        else:
            return (0.5*A_L*(self.longwave_sky(Ta) + self.longwave_ground(Tg)))

    def gha(self, u): #u = wind speed m/s
        'boundary layer conductance for hear assuming d=diameter, Table 7.5 Campbell and Norman'
        return 1.4*(0.135*math.sqrt(u/self.D))
    
    def gr(self, Ta, cp):
        'radiative conductance of the animal, Ta needs to be at height of animal'
        return ((4*STEFAN_BOLTZMANN)*((Ta +273.15)**3.))/cp
      
    def operative(self, Ta, julian, hour, s, A_S, tau, Tg, A_L, cp, u):
        return Ta + ((self.radiation_abs(julian, hour, s, A_S, tau, Ta, Tg, A_L) - self.E_S*STEFAN_BOLTZMANN*Ta**4) / (cp*(self.gr(Ta, cp)+self.gha(u))))
    
    def Tb2(self, Te, Tb1, time_at_temp, time_constant):
        return Te + ((Tb1-Te)*(math.exp(-time_at_temp/time_constant)))

    # def activity_status_5C(self, Te, Tb1, time_at_temp, time_constant):
    #     activity_status = [1 if (Tpref_min <= shade <= Tpref_max or Tpref_min <= sun <= Tpref_max) else 0 for shade, sun in zip(tb_shade, tb_sun)]


windspeeds = [0.1]# [0.1,1.0,2.0,3.0]
time_at_temp=5.

species = pd.read_csv(join(ROOT_DIR, 'parameters/input.csv')) 
# hourly_results = pd.DataFrame(columns = ['species','scenario_group','julian','hour','tpref_mean','Rabs_sun','Rabs_shade','Te_sun','Te_shade','Tb_sun','Tb_shade','activity_status_5C','activity_status_25C','activity_status_skewed_5C','activity_status_skewed_10C', 'ro','rcm','growth_k','growth_linf'])
activity_data = []
# hourly_results = pd.DataFrame(columns = ['species','scenario_group','julian','hour','Rabs_sun','Rabs_shade','Te_sun','Te_shade','Tb_sun', 'Tb_shade'])


try:
    for i in range(len(species)):
        ectotherm = Individual(species.type[i],species.spp[i],species.lizard_location[i], scenarios[i],species.latitude[i],species.longitude[i],species.altitude[i],species.mass[i],species.length[i],species.width[i],species.emissivity[i],species.tpref_mean[i])

        loaded_frame = ectotherm.dynamic_frame_load()

        previous_tb_timestep_sun = 5 #replacing Tb1 dummy start value
        previous_tb_timestep_shade = 5

        for index, row in loaded_frame.iterrows():
            # if (index == 2): #12Jan2020 , include when wanting to check if code works but not generate full dataset
            #     break #12Jan2020 , include when wanting to check if code works but not generate full dataset
            # failedhour=index
            julian = row['julian']
            hour = row['hour']
            Ta_sun = row['Ta_sun']
            Ta_shade = row['Ta_shade']
            sun_D0cm = row['sun_D0cm']
            shade_D0cm = row['shade_D0cm']
            Rabs_sun = ectotherm.radiation_abs(julian, hour, 1., A_S, tau, Ta_sun, sun_D0cm, A_L)
            Rabs_shade = ectotherm.radiation_abs(julian, hour, 0., A_S, tau, Ta_shade, shade_D0cm, A_L)

            Te_sun=ectotherm.operative(Ta_sun, julian, hour, s, A_S, tau, sun_D0cm, A_L, cp, u)
            Te_shade=ectotherm.operative(Ta_shade, julian, hour, s, A_S, tau, shade_D0cm, A_L, cp, u)

            if Te_sun >= previous_tb_timestep_sun:
                time_constant=math.exp(0.72+0.36*log(ectotherm.MASS))
            elif Te_shade >= previous_tb_timestep_shade:
                time_constant=math.exp(0.72+0.36*log(ectotherm.MASS))
            elif Te_sun <= previous_tb_timestep_sun:
                time_constant=math.exp(0.42+0.44*log(ectotherm.MASS))
            elif Te_shade <= previous_tb_timestep_shade:
                time_constant=math.exp(0.42+0.44*log(ectotherm.MASS))
            else:
                print('warning: time_constant shit is fucked')

            Tb_sun= ectotherm.Tb2(Te_sun, previous_tb_timestep_sun, time_at_temp, time_constant)
            Tb_shade= ectotherm.Tb2(Te_shade, previous_tb_timestep_shade, time_at_temp, time_constant)
            previous_tb_timestep_sun = Tb_sun
            previous_tb_timestep_shade = Tb_shade

            # activity_status_5C = 0. if (Tb_sun > (ectotherm.tpref_mean+5.0)) & (Tb_shade > (ectotherm.tpref_mean+5.0)) | (Tb_sun < (ectotherm.tpref_mean-5.0)) else 1. #original
            # activity_status_25C = 0. if (Tb_sun > (ectotherm.tpref_mean+2.5)) & (Tb_shade > (ectotherm.tpref_mean+2.5))| (Tb_sun < (ectotherm.tpref_mean-2.5)) else 1. #original

            #activity_status_5C = 1. if (8. < hour < 18.) & (Tb_sun < (ectotherm.tpref_mean+5.0)) & (Tb_shade < (ectotherm.tpref_mean+5.0)) | (Tb_sun > (ectotherm.tpref_mean-5.0)) else 0.
            #activity_status_25C = 1. if (8. < hour < 18.) & (Tb_sun < (ectotherm.tpref_mean+2.5)) & (Tb_shade < (ectotherm.tpref_mean+2.5)) | (Tb_sun > (ectotherm.tpref_mean-2.5)) else 0.
            activity_status_5C = None #declaring it so it exists.. scope issue
            activity_status_25C = None



            if (480. < hour < 1080.):
                if (Tb_sun < (ectotherm.tpref_mean+5.0)) & (Tb_shade < (ectotherm.tpref_mean+5.0)) | (Tb_sun > (ectotherm.tpref_mean-5.0)):
                    activity_status_5C = 1.
                else:
                    activity_status_5C = 0.
            if (480. < hour < 1080.):
                if (Tb_sun < (ectotherm.tpref_mean+2.5)) & (Tb_shade < (ectotherm.tpref_mean+2.5)) | (Tb_sun > (ectotherm.tpref_mean-2.5)):
                    activity_status_25C = 1.
                else:
                    activity_status_25C = 0.


            



            ####

#### APRIL 15 laptop

            #activity_status_5C = 0. if Tb_sun > (ectotherm.tpref_mean+5.0) or Tb_shade < (ectotherm.tpref_mean-5.0) else 1. 
            #activity_status_25C = 0. if Tb_sun > (ectotherm.tpref_mean+2.5) or Tb_shade < (ectotherm.tpref_mean-2.5) else 1.
            # activity_status_skewed_5C = [0. if Tb_sun > (ectotherm.tpref_mean+1.25) or Tb_shade < (ectotherm.tpref_mean-3.75) else 1.]
            # activity_status_skewed_10C = [0. if Tb_sun > (ectotherm.tpref_mean+2.5) or Tb_shade < (ectotherm.tpref_mean-7.5) else 1.]

            # activity_status_5C = [1 if ((Tpref_mean-2.5) <= Tb_shade <= (Tpref_mean+2.5) or (Tpref_mean-2.5) <= Tb_sun <= (Tpref_mean+2.5) else 0 for shade, sun in zip(tb_shade, tb_sun)]

            activity_data.append([species.spp[i], scenarios[i], julian, hour, species.tpref_mean[i], Rabs_sun, Rabs_shade, Te_sun, Te_shade, Tb_sun, Tb_shade, activity_status_5C, activity_status_25C,species.ro[i],species.rcm[i],species.growth_k[i],species.growth_linf[i]])
            # if (index == 2): #comment this line out when generating full results
            #     break #comment this line out when generating full results

    dataframe = pd.DataFrame(activity_data, columns = ['species','scenario','julian','hour','Tpref_mean','Rabs_sun','Rabs_shade','Te_sun','Te_shade','Tb_sun','Tb_shade','activity_status_5C','activity_status_25C','ro','rcm','growth_k','growth_linf'])
    with open(join(dirname(dirname(__file__)), 'output/results.csv'), 'w') as f:
        dataframe.to_csv(f, header=True)
except Exception as e:
    print(e)



#summarize results
hourly= pd.read_csv(join(ROOT_DIR, 'output/results.csv'))
# daily_results = hourly.groupby(['species','scenario','julian','hour','Tpref_mean','Rabs_sun','Rabs_shade','Te_sun','Te_shade','Tb_sun','Tb_shade','activity_status_5C','activity_status_25C','ro','rcm','growth_k','growth_linf'],as_index=False)
# daily_results = hourly.groupby(['scenario'],as_index=False)
# daily_df = pd.DataFrame([OrderedDict([
#                  ('activity_status_5C' , hourly.groupby(by=["scenario"])["activity_status_5C"].sum().reset_index(name='lauren')),
#                  ('activity_status_25C', hourly.groupby(by=["scenario"])["activity_status_25C"].sum().reset_index(name='jake')),
#                 #  ('Te_sun', statistics.mean),
#                 #  ('Te_shade' , statistics.mean),
#                 #  ('Tb_sun' , statistics.mean),
#                 #  ('Tb_shade',statistics.mean),
#                 ])], columns=["sum_activity_status_5C", "sum_activity_status_25C"])
resultant_df = pd.merge(hourly.groupby(by=["scenario"])["activity_status_5C"].sum().reset_index(name='sum_activity_status_5C'), hourly.groupby(by=["scenario"])["activity_status_25C"].sum().reset_index(name='sum_activity_status_25C'), how='inner', on=['scenario'])
resultant_df.to_csv(join(ROOT_DIR, 'output/daily_results.csv'),index = False)




# %%
# windspeeds = [0.1]# [0.1,1.0,2.0,3.0]
# time_at_temp=5.

# species = pd.read_csv('/Users/laurenneel/Desktop/input_test2.csv')
# hourly_results = pd.DataFrame(columns = ['species','scenario_group','julian','hour','Rabs_sun','Rabs_shade'])


# # hourly_results = pd.DataFrame(columns = ['species','scenario_group','julian','hour','Rabs_sun','Rabs_shade','Te_sun','Te_shade','Tb_sun', 'Tb_shade'])
# # hourly_results = []
# try:
#     # print(1/0)
#     for i in range(len(species)):
#         ectotherm = Individual(species.type[i],species.spp[i],species.lizard_location[i], scenarios[i],species.latitude[i],species.longitude[i],species.altitude[i],species.mass[i],species.length[i],species.width[i],species.emissivity[i])
#         loaded_frame = ectotherm.dynamic_frame_load()

#         # print(len(loaded_frame))
#         # previous_tb_timestep_sun = 5 #replacing Tb1 dummy start value
#         # previous_tb_timestep_shade = 5


#         for index, row in loaded_frame.iterrows():
#             julian = row['julian']
#             hour = row['hour']
#             Ta_sun = row['Ta_sun']
#             Ta_shade = row['Ta_shade']
#             sun_D0cm = row['sun_D0cm']
#             shade_D0cm = row['shade_D0cm']
#             Rabs_sun = ectotherm.radiation_abs(julian, hour, 1., A_S, tau, Ta_sun, sun_D0cm, A_L)
#             Rabs_shade = ectotherm.radiation_abs(julian, hour, 0., A_S, tau, Ta_shade, shade_D0cm, A_L)
#             dataframe = pd.DataFrame([[species.spp[i], scenarios[i], julian, hour, Rabs_sun, Rabs_shade]], columns = ['species','scenario','julian','hour','Rabs_sun','Rabs_shade'])
#             hourly_results.append(dataframe)
# hourly_results.to_csv('testdata23Nov.csv',columns=['species','scenario','julian','hour','Rabs_sun','Rabs_shade'],index=False)

            
            
            
# except Exception as e:
#     print(e)

            
# # hourly_results.append(dataframe)
 
# #             with open('results.csv', 'a') as f:
# #                 dataframe.to_csv(f, header=True)
# #                 hourly_results.append(dataframe)


# %%
# windspeeds = [0.1]# [0.1,1.0,2.0,3.0]
# time_at_temp=5.

# species = pd.read_csv(join(ROOT_DIR, 'parameters/input.csv'))
# hourly_results = pd.DataFrame(columns = ['species','scenario_group','julian','hour','Rabs_sun','Rabs_shade'])

# test=[]
# # hourly_results = pd.DataFrame(columns = ['species','scenario_group','julian','hour','Rabs_sun','Rabs_shade','Te_sun','Te_shade','Tb_sun', 'Tb_shade'])
# try:
#     # print(1/0)
#     for i in range(len(species)):
#         ectotherm = Individual(species.type[i],species.spp[i],species.lizard_location[i], scenarios[i],species.latitude[i],species.longitude[i],species.altitude[i],species.mass[i],species.length[i],species.width[i],species.emissivity[i])
#         loaded_frame = ectotherm.dynamic_frame_load()

#         # print(len(loaded_frame))
#         # previous_tb_timestep_sun = 5 #replacing Tb1 dummy start value
#         # previous_tb_timestep_shade = 5

#         for index, row in loaded_frame.iterrows():
#             julian = row['julian']
#             hour = row['hour']
#             Ta_sun = row['Ta_sun']
#             Ta_shade = row['Ta_shade']
#             sun_D0cm = row['sun_D0cm']
#             shade_D0cm = row['shade_D0cm']

#             Rabs_sun = ectotherm.radiation_abs(julian, hour, 1., A_S, tau, Ta_sun, sun_D0cm, A_L)
#             Rabs_shade = ectotherm.radiation_abs(julian, hour, 0., A_S, tau, Ta_shade, shade_D0cm, A_L)
#             test.append(Rabs_shade)

#             # dataframe = pd.DataFrame([[species.spp[i], scenarios[i], julian, hour, Rabs_sun, Rabs_shade]], columns = ['species','scenario','julian','hour','Rabs_sun','Rabs_shade'])
#             #
#             #
#             # with open('results.csv', 'a') as f:
#             #     dataframe.to_csv(f, header=True)
#             #     hourly_results.append(dataframe)
# except Exception as e:
#     print(e)


# %%


# Te_sun=ectotherm.operative(Ta_sun, julian, hour, s, A_S, tau, sun_D0cm, A_L, cp, u)
#             Te_shade=ectotherm.operative(Ta_shade, julian, hour, s, A_S, tau, shade_D0cm, A_L, cp, u)

#             if Te_sun >= previous_tb_timestep_sun:
#                 time_constant=math.exp(0.72+0.36*log(ectotherm.MASS))
#             elif Te_shade >= previous_tb_timestep_shade:
#                 time_constant=math.exp(0.72+0.36*log(ectotherm.MASS))
#             elif Te_sun <= previous_tb_timestep_sun:
#                 time_constant=math.exp(0.42+0.44*log(ectotherm.MASS))
#             elif Te_shade <= previous_tb_timestep_shade:
#                 time_constant=math.exp(0.42+0.44*log(ectotherm.MASS))
#             else:
#                 print('warning: time_constant shit is fucked')
#             Tb_sun= ectotherm.Tb2(Te_sun, previous_tb_timestep_sun, time_at_temp, time_constant)
#             Tb_shade= ectotherm.Tb2(Te_shade, previous_tb_timestep_shade, time_at_temp, time_constant)
#             previous_tb_timestep_sun = Tb_sun
#             previous_tb_timestep_shade = Tb_shade
#             dataframe = pd.DataFrame([[species.spp[i], scenarios[scenario_val], julian, hour, Rabs_sun, Rabs_shade]], columns = ['species','scenario','julian','hour','Rabs_sun','Rabs_shade'])


# #             dataframe = pd.DataFrame([[species.spp[i],scenarios[scenario_val],julian,hour,Rabs_sun,Rabs_shade,Te_sun,Te_shade,Tb_sun,Tb_shade]],columns = ['species','scenario','julian','hour','Rabs_sun','Rabs_shade','Te_sun','Te_shade','Tb_sun','Tb_shade'])
# #             with open ('results.csv', 'a') as f:
# #                 dataframe.to_csv(f, header=True)
#             hourly_results.append(dataframe)



# print(len(hourly_results))
# print(hourly_results)


#print(d.loc[0,:])


# %%


# #dataframe with species parameters

# windspeeds = [0.1]# [0.1,1.0,2.0,3.0]
# time_at_temp=5.

# species = pd.read_csv('/Users/laurenneel/Desktop/input_test2.csv')
# test=[]

# hourly_results = pd.DataFrame(columns = ['species','scenario_group','julian','hour','Rabs_sun','Rabs_shade','Te_sun','Te_shade','Tb_sun', 'Tb_shade'])
# # hourly_results = []
# for i in range(len(species)):
#     for scenario_val in range(len(scenarios)):
# #         print(scenario_val)
#         ectotherm = Individual(species.type[i],species.spp[i],species.lizard_location[i],scenarios[scenario_val],species.latitude[i],species.longitude[i],species.altitude[i],species.mass[i],species.length[i],species.width[i],species.emissivity[i])
#         loaded_frame = ectotherm.dynamic_frame_load()
# #         print(len(loaded_frame))
# #         previous_tb_timestep_sun = 5 #replacing Tb1 dummy start value
# #         previous_tb_timestep_shade = 5

    
#         for index, row in loaded_frame.iterrows():
#             julian = row['julian']
#             hour = row['hour']
#             Ta_sun = row['Ta_sun']
#             Ta_shade = row['Ta_shade']
#             sun_D0cm = row['sun_D0cm']
#             shade_D0cm = row['shade_D0cm']
            
        
#             Rabs_sun = ectotherm.radiation_abs(julian, hour, 1., A_S, tau, Ta_sun, sun_D0cm, A_L)
#             Rabs_shade = ectotherm.radiation_abs(julian, hour, 0., A_S, tau, Ta_shade, shade_D0cm, A_L)
#             test.append(Rabs_shade)
# print(test)
            
# #             Te_sun=ectotherm.operative(Ta_sun, julian, hour, s, A_S, tau, sun_D0cm, A_L, cp, u)
# #             Te_shade=ectotherm.operative(Ta_shade, julian, hour, s, A_S, tau, shade_D0cm, A_L, cp, u)

# #             if Te_sun >= previous_tb_timestep_sun:
# #                 time_constant=math.exp(0.72+0.36*log(ectotherm.MASS))
# #             elif Te_shade >= previous_tb_timestep_shade:
# #                 time_constant=math.exp(0.72+0.36*log(ectotherm.MASS))
# #             elif Te_sun <= previous_tb_timestep_sun:
# #                 time_constant=math.exp(0.42+0.44*log(ectotherm.MASS))
# #             elif Te_shade <= previous_tb_timestep_shade:
# #                 time_constant=math.exp(0.42+0.44*log(ectotherm.MASS))
# #             else:
# #                 print('warning: time_constant shit is fucked')
# #             Tb_sun= ectotherm.Tb2(Te_sun, previous_tb_timestep_sun, time_at_temp, time_constant)
# #             Tb_shade= ectotherm.Tb2(Te_shade, previous_tb_timestep_shade, time_at_temp, time_constant)
# #             previous_tb_timestep_sun = Tb_sun
# #             previous_tb_timestep_shade = Tb_shade
# #             dataframe = pd.DataFrame([[species.spp[i], scenarios[scenario_val], julian, hour, Rabs_sun, Rabs_shade]], columns = ['species','scenario','julian','hour','Rabs_sun','Rabs_shade'])


# # #             dataframe = pd.DataFrame([[species.spp[i],scenarios[scenario_val],julian,hour,Rabs_sun,Rabs_shade,Te_sun,Te_shade,Tb_sun,Tb_shade]],columns = ['species','scenario','julian','hour','Rabs_sun','Rabs_shade','Te_sun','Te_shade','Tb_sun','Tb_shade'])
# # #             with open ('results.csv', 'a') as f:
# # #                 dataframe.to_csv(f, header=True)
# #             hourly_results.append(dataframe)
    

    
# # print(len(hourly_results))
# # print(hourly_results)


# #print(d.loc[0,:])
        
            
# # hourly_results.to_csv('Desktop/results.csv',columns=['species','scenario_group','julian','hour','Rabs_sun','Rabs_shade'])

# #             dataframe.join([pd.DataFrame([species.spp[i],scenarios[scenario_val],julian,hour,Rabs_sun,Rabs_shade,Te_sun,Te_shade,Tb_sun,Tb_shade],columns = ['species','scenario','julian','hour','Rabs_sun','Rabs_shade','Te_sun','Te_shade','Tb_sun','Tb_shade'])])


               
                
#                 #Te_sun = ectotherm.operative(julian_list[j], hour_list[h], 1.0, A_S, tau, A_L, cp, u)
#                 #time_constant= math.exp(0.72+0.36*log(species.mass[i])), np.where(Te_sun >= Tb1[0])
#                 #time_constant=math.exp(0.42+0.44*log(species.mass[i])), np.where(Te_sun < Tb1[0])
                
# #print(len(Te_sun))               
                
# #                 if Te_sun >= Tb1[0]:
# #                     time_constant= math.exp(0.72+0.36*log(species.mass[i]))
# #                 else:
# #                     time_constant=math.exp(0.42+0.44*log(species.mass[i]))
# #                 Tb_sun = ectotherm.Tb2(Te_sun,Tb1[0],time_at_temp, time_constant)
# #             dataframe = pd.DataFrame([[species.spp[i],scenarios[scenario_val],julian,hour,Rabs_sun]],columns = ['species','study_group','julian','hour','Rabs_sun'])
# #             hourly_results.append(dataframe)
# # hourly_results.to_csv('Desktop/hourly_v14_standard.csv',columns=['species','study_group','julian','hour','Rabs_sun'])

