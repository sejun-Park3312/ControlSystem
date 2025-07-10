import time
import numpy as np
import pandas as pd

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


    def SaveData(self, Data, FileName):
        FileName += '.xlsx'
        TimeStamp = Data["Time"]["TimeStamp"]
        Value = Data["Value"]

        ExelData = {'Time': TimeStamp}
        for key in Value:
            ExelData[key] = Value[key]

        df = pd.DataFrame(ExelData)
        df.to_excel(FileName, index=False)
        print(f"Data Saved!")