import ftplib


class ABB_Robot():
    def __init__(self):
        self.host = '192.168.30.4'
        self.user = 'anonymous'
        self.password = 'anonymous@'

        # Powder conditions
        self.carrier_gas = 3
        self.stirrer = 20
        self.turntable = 20

        self.power = 1200 # define by layer
        self.track_speed = 8

        self.speed = 'vl' # use speeddata v6
        self.zone = 'z0'

        self.travel_speed = 'v50'
        self.travel_zone = 'z0'

        self.tool = [[215.7, -22.4, 473.8], [0.50, 0.0, -0.8660254, 0.0]] # Tool pose
        self.workobject = [[1655, -87, 932], [1, 0, 0, 0]] # Work Object pose

    def path2rapid(self, path):
        RAPID_TEMPLATE  = 'MODULE Etna\n'
        RAPID_TEMPLATE += '\n'
        RAPID_TEMPLATE += '    PERS tooldata toolEtna:=%(tool)s;\n'
        RAPID_TEMPLATE += '    PERS wobjdata wobjEtna:=%(wobj)s;\n'
        RAPID_TEMPLATE += '\n'
        RAPID_TEMPLATE += '    VAR triggdata laserON;\n'
        RAPID_TEMPLATE += '    VAR triggdata laserOFF;\n'
        RAPID_TEMPLATE += '\n'
        RAPID_TEMPLATE += '    CONST speeddata vl:=[%(speed)i,500,5000,1000];\n' #CONST speeddata v15 := [15,500,5000,1000];
        RAPID_TEMPLATE += '\n'
        RAPID_TEMPLATE += '    %(targets)s\n'
        RAPID_TEMPLATE += '\n'
        RAPID_TEMPLATE += 'PROC cladding()\n'
        RAPID_TEMPLATE += '    %(moves)s\n'
        RAPID_TEMPLATE += 'ENDPROC\n'
        RAPID_TEMPLATE += '\n'
        RAPID_TEMPLATE += 'PROC mainEtna()\n'
        RAPID_TEMPLATE += '    Set Do_RF_MainOn;\n'
        RAPID_TEMPLATE += '    Set Do_RF_StandByOn;\n'
        RAPID_TEMPLATE += '    WaitDI DI_RF_LaserBeamReady,1;\n'
        RAPID_TEMPLATE += '    WaitDI DI_RF_GeneralFault,0;\n'
        RAPID_TEMPLATE += '\n'
        RAPID_TEMPLATE += '    SetGO GO_Program_Rf, 0;\n' # set the program for control of laser power - prog 5
        RAPID_TEMPLATE += '    WaitTime 1;\n'
        RAPID_TEMPLATE += '    !SetGO GoLDL_Pwr3, %(power)i;\n' # set the laser power - 2200 W
        RAPID_TEMPLATE += '\n'
        RAPID_TEMPLATE += '    TriggIO laserON, 0\DOp:=Do_RF_ExterGate, 1;\n'
        RAPID_TEMPLATE += '    TriggIO laserOFF, 0\DOp:=Do_RF_ExterGate, 0;\n'
        RAPID_TEMPLATE += '\n'
        RAPID_TEMPLATE += '    Set DoWeldGas;\n'
        RAPID_TEMPLATE += '    !MedicoatL2 "OFF", 5, 20, 7.5;\n' # gas de arrastre, stirrer, turntable
        RAPID_TEMPLATE += '    MedicoatL1 "OFF", %(carrier)i, %(stirrer)i, %(turntable)i;\n'
        RAPID_TEMPLATE += '\n'
        RAPID_TEMPLATE += '    ConfL \Off;\n'
        RAPID_TEMPLATE += '\n'
        RAPID_TEMPLATE += '    MoveL [[0.0,0.0,100.0],[1.0,0.0,0.0,0.0],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],v80,z0,toolEtna\WObj:=wobjEtna;\n'
        RAPID_TEMPLATE += '\n'
        RAPID_TEMPLATE += '    cladding;\n'
        RAPID_TEMPLATE += '\n'
        RAPID_TEMPLATE += '    MoveL [[0.0,0.0,100.0],[1.0,0.0,0.0,0.0],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],v80,z0,toolEtna\WObj:=wobjEtna;\n'
        RAPID_TEMPLATE += '\n'
        RAPID_TEMPLATE += '    Reset doMdtPL2On;\n'
        RAPID_TEMPLATE += '    Reset doMdtPL1On;\n'
        RAPID_TEMPLATE += '    Reset DoWeldGas;\n'
        RAPID_TEMPLATE += '\n'
        RAPID_TEMPLATE += '    Reset Do_RF_StandByOn;\n'
        RAPID_TEMPLATE += '    !Reset Do_RF_MainOn;\n'
        RAPID_TEMPLATE += '\n'
        RAPID_TEMPLATE += 'ENDPROC\n'
        RAPID_TEMPLATE += '\n'
        RAPID_TEMPLATE += 'ENDMODULE\n'
        # Target points definition
        targets = ''
        for k in range(len(path)):
            p, q, b = path[k]
            targets = '\n'.join([targets, '    CONST robtarget T%i:=[[%f,%f,%f],[%f,%f,%f,%f],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];' %(k, p[0], p[1], p[2], q[3], q[0], q[1], q[2])])
        # Movement definition
        moves = '!Reset doLDLStartST;\n'
        for k in range(len(path)):
            p, q, process = path[k]
            if process:
                #moves = '\n'.join([moves, '    Set doLDLStartST;'])
                #moves = '\n'.join([moves, '    MoveL T%i,%s,%s,toolEtna\WObj:=wobjEtna;' %(k, self.speed, self.zone)])
                if k < len(path)-1 and path[k+1][2]:
                    # If the track continues in the next point. Don't OFF the laser.
                    moves = '\n'.join([moves, '    TriggL T%i,%s,laserON,%s,toolEtna\WObj:=wobjEtna;' %(k, self.speed, self.zone)])
                else:
                    moves = '\n'.join([moves, '    TriggL T%i,%s,laserOFF,%s,toolEtna\WObj:=wobjEtna;' %(k, self.speed, self.zone)])
            else:
                #moves = '\n'.join([moves, '    Reset doLDLStartST;'])
                #moves = '\n'.join([moves, '    MoveL T%i,%s,%s,toolEtna\WObj:=wobjEtna;' %(k, self.travel_speed, self.travel_zone)])
                moves = '\n'.join([moves, '    TriggL T%i,%s,laserON,%s,toolEtna\WObj:=wobjEtna;' %(k, self.travel_speed, self.travel_zone)])
        moves = '\n'.join([moves, '\n!Reset doLDLStartST;'])

        tool = '[TRUE,[[%.1f,%.1f,%.1f],[%f,%f,%f,%f]],[20,[70,30,123.5],[0,0,1,0],1,0,1]]' %(self.tool[0][0], self.tool[0][1], self.tool[0][2], self.tool[1][0], self.tool[1][1], self.tool[1][2], self.tool[1][3])
        wobj = '[FALSE,TRUE,"",[[%.1f,%.1f,%.1f],[%f,%f,%f,%f]],[[0,0,0],[1,0,0,0]]]' %(self.workobject[0][0], self.workobject[0][1], self.workobject[0][2], self.workobject[1][0], self.workobject[1][1], self.workobject[1][2], self.workobject[1][3])

        return RAPID_TEMPLATE %{'tool': tool,
                                'wobj': wobj,
                                'speed': self.track_speed,
                                'targets': targets,
                                'moves': moves,
                                'carrier': self.carrier_gas,
                                'stirrer': self.stirrer,
                                'turntable': self.turntable,
                                'power': self.power}

    def save_file(self, filename, routine):
        try:
            with open(filename, 'w') as f:
                f.writelines(routine)
        except IOError:
            pass

    def upload_file(self, filename, directory):
        try:
            ftp = ftplib.FTP(self.host, self.user, self.password, timeout=1)
            ftp.cwd(directory)
            ftp.storbinary('STOR ' + filename, open(filename,'rb'))
            print 'The file was upload sucessful!'
            print ftp.retrlines('LIST')
            ftp.quit()
        except IOError:
            print 'File transfered to the robot.'


if __name__ == '__main__':
    rob = ABB_Robot()
    path = [([0, 0, 0], [0, 0, 0, 1], False),
            ([100, 0, 0], [0, 0, 0, 1], True),
            ([100, 100, 0], [0, 0, 0, 1], True)]
    filename = 'etna.mod'
    routine = rob.path2rapid(path)
    rob.save_file(filename, routine)
    rob.upload_file(filename, 'ETNA')
    print routine
