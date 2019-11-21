# -*- coding: utf-8 -*-
"""
Copyright (C) 2019 Event-driven Perception for Robotics
Authors: Sim Bamford
This program is free software: you can redistribute it and/or modify it under 
the terms of the GNU General Public License as published by the Free Software 
Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY 
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with 
this program. If not, see <https://www.gnu.org/licenses/>.

Intended as part of bimvee (Batch Import, Manipulation, Visualisation and Export of Events etc)
Contains various functions for splitting data sets:

splitByChannel
splitByPolarity
splitDvsByLabel TODO: this could be generalised to any data type ...
"""

#%%
import numpy as np

from timestamps import rezeroTimestampsForImportedDicts

'''
2019_11_11 This function currently unused, (importIitYarp handles this internally)
but leaving it in case it should be useful
'''
def splitByChannel(inDict):
    channelNames = list(inDict['data'].keys())
    
    if len(channelNames) > 1:
        print('SplitByChannel was requested for file import ' + 
              inDict['info']['pathAndFileName'] + 
              ' but it seems this has already happened.')
        return
    channelName = channelNames[0]
    channelData = inDict['data'][channelName]
    dvs = channelData['dvs']
    if 'ch' not in dvs: 
        print('SplitByChannel was requested for file import ' + 
              inDict['info']['pathAndFileName'] + 
              ' but no channel data was found.')
        inDict['data'][channelName]
        return
    # At this point we are commited to changing the channel data field, so pop it off
    channelData = inDict['data'].pop(channelName)
    uniqueChannels = np.unique(dvs['ch'])
    if len(uniqueChannels) == 1:
        print('SplitByChannel was requested for file import ' + 
              inDict['info']['pathAndFileName'] + 
              ' but there is only one channel defined.')
        inDict['data'][uniqueChannels[0]] = inDict['data'].pop(channelName) # make sure the channel is named as it was in pol['ch']
        dvs.pop('ch') # remove the channel data, as it is now redundant 
        return
    for ch in uniqueChannels:
        booleanSelector = dvs['ch'] == ch
        inDict['data'][ch] = {
            'dvs': {
                'ts': dvs['ts'][booleanSelector],
                'x': dvs['x'][booleanSelector],
                'y': dvs['y'][booleanSelector],
                'pol': dvs['pol'][booleanSelector]}}

def splitByPolarity(inDict):
    ''' 
    receives a dict containing dvs events
    returns a dict containing two dicts, labelled 0 and 1, for the polarities found
    Although redundant, the pol field is maintained within each dict for compatibility
    '''
    outDict = {
        '0': {},
        '1': {} }
    for key in inDict:
        if key in ['ts', 'x', 'y', 'pol']:
            outDict['0'][key] = inDict[key][inDict['pol'] == 0]
            outDict['1'][key] = inDict[key][inDict['pol'] == 1]
        else:
            outDict['0'][key] = inDict[key]
            outDict['1'][key] = inDict[key]
    return outDict
    
def splitDvsByLabel(inDict):
    uniqueLbls = np.unique(inDict['lbl'])
    outDict = {}
    for uniqueLbl in uniqueLbls:
        outDict[uniqueLbl] = {}
        selectedBool = inDict['lbl'] == uniqueLbl
        for field in inDict:
            if field in ['x', 'y', 'pol', 'ts']:
                outDict[uniqueLbl][field] = inDict[field][selectedBool]
            elif field == 'lbl':
                pass
            else:
                outDict[uniqueLbl][field] = inDict[field]
    return outDict 

def dvsSelectLabels(inDict, labels):
    selectedBool = np.full_like(inDict['pol'], False)
    for label in labels:
        selectedBoolTemp = inDict['lbl'] == label
        selectedBool = selectedBool | selectedBoolTemp
    outDict = {}
    for field in inDict:
        if field in ['x', 'y', 'pol', 'ts', 'lbl']:
            outDict[field] = inDict[field][selectedBool]
        else:
            outDict[field] = inDict[field]
    return outDict    

def splitByLabelled(inDict):
    outDict = {}
    lblBool = inDict['lbl'] > -1
    if np.any(lblBool):
        outDict['dvsLbl'] = {
            'ts': inDict['ts'][lblBool],
            'x': inDict['x'][lblBool],
            'y': inDict['y'][lblBool],
            'pol': inDict['pol'][lblBool],
            'lbl': inDict['lbl'][lblBool],
                }
    if np.any(~lblBool):
        outDict['dvs'] = {
            'ts': inDict['ts'][~lblBool],
            'x': inDict['x'][~lblBool],
            'y': inDict['y'][~lblBool],
            'pol': inDict['pol'][~lblBool],
                }
    return outDict

''' 
expecting startTime, stopTime or both
If given a single dataType dict, will just cut down all arrays by masking on the ts array. 
If given a larger container, will split down all that it finds, realigning timestamps.
If the container contains an info field, then the start and stopTime params
will be added.
'''

def cropTime(inDict, **kwargs):
    # TODO: handle list case
    if 'ts' in inDict:
        ts = inDict['ts']
        startTime = kwargs.get('startTime', ts[0])
        stopTime = kwargs.get('startTime', ts[-1])
        numElements = len(ts)
        selectedIds = range(np.searchsorted(ts, startTime), np.searchsorted(ts, stopTime))
        #mask = np.bitwise_and((inDict['ts']>=startTime), (inDict['ts']<=stopTime))
        tsNew = ts[selectedIds] - startTime
        outDict = {'ts': tsNew}
        for fieldName in inDict.keys():
            if fieldName != 'ts':
                field = inDict[fieldName]
                try:
                    lenField = len(field)
                except TypeError:
                    lenField = 0
                if lenField == numElements:
                    outDict[fieldName] = field[selectedIds]
                else:
                    outDict[fieldName] = field.copy() # This might fail for certain data types
        tsOffsetOriginal = inDict.get('tsOffset', 0)
        outDict['tsOffset'] = tsOffsetOriginal - startTime
        return outDict
    elif 'info' in inDict:
        outDict = {'info': inDict['info'].copy(),
                   'data': {}}
        for channelName in inDict['data'].keys():
            outDict['data'][channelName] = {}
            for dataTypeName in inDict['data'][channelName].keys():
                outDict['data'][channelName][dataTypeName] = cropTime(inDict['data'][channelName][dataTypeName])                
        return rezeroTimestampsForImportedDicts(outDict)
    else:
        raise ValueError('input not in the form expected')