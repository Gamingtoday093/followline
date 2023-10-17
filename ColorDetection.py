from pybricks.ev3devices import ColorSensor

EQUALS_TOLERENCE = 30
LINE_BIAS = 1 / 3

class ColorCompareUsage:
    def __init__(self, useRGB: bool, rgb: tuple[int, int, int], useReflection: bool, reflection: int, useAmbient: bool, ambient: int) -> None:
        self.UseRGB = useRGB
        self.RGB = rgb
        self.UseReflection = useReflection
        self.Reflection = reflection
        self.UseAmbient = useAmbient
        self.Ambient = ambient

class ColorHDR:
    def __init__(self, rgb: tuple[int, int, int], reflection: int, ambient: int):
        self.Color = [rgb[0], rgb[1], rgb[2], reflection, ambient]

    @classmethod
    def fromColorSensor(cls, sensor: ColorSensor, compare: ColorCompareUsage = None):
        rgb = (0, 0, 0)
        if compare is None or compare.UseRGB:
            rgb = sensor.rgb()
        else:
            rgb = compare.RGB
        reflection = 0
        if compare is None or compare.UseReflection:
            reflection = sensor.reflection()
        else:
            reflection = compare.Reflection
        ambient = 0
        if compare is None or compare.UseAmbient:
            ambient = sensor.ambient()
            if ambient < 0:
                ambient = 0
        else:
            ambient = compare.Ambient

        return cls(rgb, reflection, ambient)

    def updateFromColorSensor(self, sensor: ColorSensor, compare: ColorCompareUsage = None):
        rgb = None
        if compare is None or compare.UseRGB:
            rgb = sensor.rgb()
        else:
            rgb = compare.RGB
        
        self.Color[0] = rgb[0]
        self.Color[1] = rgb[1]
        self.Color[2] = rgb[2]

        reflection = 0
        if compare is None or compare.UseReflection:
            reflection = sensor.reflection()
        else:
            reflection = compare.Reflection

        self.Color[3] = reflection

        ambient = 0
        if compare is None or compare.UseAmbient:
            ambient = sensor.ambient()
            if ambient < 0:
                ambient = 0
        else:
            ambient = compare.Ambient
        
        self.Color[4] = ambient

    @staticmethod
    def equals(color1: int, color2: int) -> bool:
        #print(str(color1) + " - " + str(color2))
        if color1 < 0 or color2 < 0:
            print(str(color1) + " - " + str(color2))
            return True
        offset = color1 - color2
        return offset <= EQUALS_TOLERENCE and offset >= -EQUALS_TOLERENCE
    
    #Detectable colors
    def __eq__(self, __value: object) -> bool:
        if not isinstance(__value, ColorHDR):
            return False
        
        #for c in range(len(self.rgb())):
        #    if not ColorHDR.equals(self.rgb()[c], __value.rgb()[c]):
        #        return False
        
        if not ColorHDR.equals(self.reflection(), __value.reflection()):
            return False

        #for c in range(len(self.Color)):
            #offset = self.Color[c] - __value.Color[c]
            #if offset > EQUALS_TOLERENCE or offset < -EQUALS_TOLERENCE:
            #    return False
        #    if not ColorHDR.equals(self.Color[c], __value.Color[c]):
        #        return False

        return True
    
    def almostEqual(self, sensorColor: object, other: object) -> bool:
        return self.__eq__(sensorColor)
        colAverage = float((self.average() + other.average()) / 2) - LINE_BIAS
        #print(str(sensorColor.Color) + " (" + str(sensorColor.average()) + "), " + str(self.Color) + ", " + str(other.Color) + " (" + str(colAverage) + ")")
        if sensorColor.average() <= colAverage:
            return False
        return True

    # Average color
    
    def add(self, value: object):
        for c in range(len(self.Color)):
            self.Color[c] += value.Color[c]
    
    def divide(self, value: int):
        for c in range(len(self.Color)):
            self.Color[c] /= value

    def average(self) -> float:
        tot = 0
        for c in self.rgb():  # self.Color
            tot += c
        tot /= len(self.rgb())  # self.Color
        return tot

    def rgb(self) -> tuple[int, int, int]:
        """ rgb() -> tuple[int, int, int]

            Returns:
            Tuple of reflections for red, green, and blue light, each
            ranging from 0.0% (no reflection) to 100.0% (high reflection).
        """
        return (self.Color[0], self.Color[1], self.Color[2])
    
    def reflection(self) -> int:
        """ reflection() -> int %:
        
            Returns:
            Reflection, ranging from 0% (no reflection) to
            100% (high reflection).
        """
        return self.Color[3]
    
    def ambient(self) -> int:
        """ ambient() -> int: %

            Returns:
            Ambient light intensity, ranging from 0% (dark)
            to 100% (bright).
        """
        return self.Color[4]

    def ValidColor(self):
        return self.ambient() > 0

    def NoneColor(self):
        for c in range(len(self.Color) - 2): # Skip Ambient
            if self.Color[c] != 0:
                return False
        return True

    @staticmethod
    def compare(color1, color2) -> ColorCompareUsage:
        """ compare(ColorHDR, ColorHDR) -> ColorCompareUsage: bool, bool, bool

            Returns:
            ColorCompareUsage which consists of 3 bools that describe what sensors are important
        """
        useRGB = False
        useReflection = False
        useAmbient = False

        for c in range(len(color1.rgb())):
            if not ColorHDR.equals(color1.rgb()[c], color2.rgb()[c]):
                useRGB = True
    
        useReflection = not ColorHDR.equals(color1.reflection(), color2.reflection())

        useAmbient = not ColorHDR.equals(color1.ambient(), color2.ambient()) and (not useRGB or not useReflection)

        if not useRGB and not useReflection and not useAmbient:
            useRGB = True
            print("Failed to find any different comparable Colors!")
        return ColorCompareUsage(useRGB, color1.rgb(), useReflection, color1.reflection(), useAmbient, color1.ambient())
        