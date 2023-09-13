from pybricks.ev3devices import ColorSensor

EQUALS_TOLERENCE = 3

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
        self.Color = (rgb[0], rgb[1], rgb[2], reflection, ambient)

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
        else:
            ambient = compare.Ambient
        
        return cls(rgb, reflection, ambient)

    @staticmethod
    def equals(color1: int, color2: int) -> bool:
        offset = color1 - color2
        return offset <= EQUALS_TOLERENCE and offset >= -EQUALS_TOLERENCE
    
    def __eq__(self, __value: object) -> bool:
        if not isinstance(__value, ColorHDR):
            return False
        
        print(self.Color)
        for c in range(len(self.Color)):
            offset = self.Color[c] - __value.Color[c]
            if offset > EQUALS_TOLERENCE or offset < -EQUALS_TOLERENCE:
                return False
            #if not ColorHDR.equals(self.Color[c], __value.Color[c]):
            #    return False

        return True
    
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
    
    @staticmethod
    def compare(color1, color2) -> ColorCompareUsage:
        """ compare(ColorHDR, ColorHDR) -> ColorCompareUsage: bool, bool, bool

            Returns:
            
        """
        useRGB = False
        useReflection = False
        useAmbient = False

        for c in range(len(color1.rgb())):
            if not ColorHDR.equals(color1.rgb()[c], color2.rgb()[c]):
                useRGB = True
    
        useReflection = not ColorHDR.equals(color1.reflection(), color2.reflection())

        useAmbient = not ColorHDR.equals(color1.ambient(), color2.ambient()) and (not useRGB or not useReflection)

        return ColorCompareUsage(useRGB, color1.rgb(), useReflection, color1.reflection(), useAmbient, color1.ambient())
        