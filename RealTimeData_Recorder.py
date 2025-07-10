import time
import numpy as np
import pandas as pd
import os

class RealTimeData_Recorder:
    def __init__(self):
        self.Data = {}


    def np2list(self, Value):
        if isinstance(Value, np.ndarray):
            return Value.tolist()
        else:
            return Value


    def DefineData(self, DataName, Keys):
        if DataName not in self.Data:
            self.Data[DataName] = {}
            self.Data[DataName]['Value'] = {key: [] for key in Keys}
            self.Data[DataName]['Time'] = {'TimeStamp': [], 'StartTime': None}

            print(self.Data[DataName])
            print(f"Data Storage '{DataName}' Created!")
        else:
            print(f"Data Storage '{DataName}' Already Exists!")


    def AppendData(self, DataName, Value):
        Value = self.np2list(Value)

        Keys = list(self.Data[DataName]['Value'].keys())
        for key, value in zip(Keys, Value):
            self.Data[DataName]['Value'][key].append(value)

        if self.Data[DataName]['Time']['StartTime']:
            self.Data[DataName]['Time']['TimeStamp'].append(time.time())
        else:
            StartTime = time.time()
            self.Data[DataName]['Time']['StartTime'] = StartTime
            self.Data[DataName]['Time']['TimeStamp'].append(time.time())


    def SaveData(self, DataName, FileName, SavePath = None):

        if SavePath is None:
            SavePath = './Results/'
        os.makedirs(SavePath, exist_ok=True)
        FullFileName = os.path.join(SavePath, FileName + '.xlsx')

        Data = self.Data[DataName]
        StartTime = Data['Time']['StartTime']
        Data['Time']['TimeStamp'] = [t - StartTime for t in Data['Time']['TimeStamp']]

        TimeStamp = Data["Time"]["TimeStamp"]
        Value = Data["Value"]

        ExelData = {'Time': TimeStamp}
        for key in Value:
            ExelData[key] = Value[key]

        df = pd.DataFrame(ExelData)
        df.to_excel(FullFileName, index=False)
        print(f"{FileName} Saved!")