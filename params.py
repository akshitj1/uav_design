import utils

proto_scale = 2.0/3

class MissionParams:
    def __init__(self) -> None:
        # platform independent requirements
        self.payload_mass = 1.5 * proto_scale**3
        delivery_time = 10*60
        # includes only restaurant to drop location, excludes charge station distances
        self.delivery_distance = 30*1e3 * proto_scale
        # charge->pickup->drop->charge = 3 climb up + 3 climb down
        self.num_climbs = 3
        self.num_lands = self.num_climbs
        num_charge_points = 90
        delivery_service_grid_edge = self.delivery_distance  # ideally should be diagonal
        delivery_service_area = utils.square_area(delivery_service_grid_edge)
        charge_point_service_area = delivery_service_area/num_charge_points
        # max. nearest charge point is center to vertex in square
        self.charge_point_distance = utils.diagonal_from_square_area(
            charge_point_service_area)/2
        self.cruise_distance = 2*self.charge_point_distance+self.delivery_distance
        self.cruise_height = 150 * proto_scale


class UavParams:
    def __init__(self, mission_params: MissionParams) -> None:
        self.atmospheric_density = 1.1225
        self.gravity = 9.81

        self.payload_mass = mission_params.payload_mass
        # dominos medium size pizza
        # self.payload_area = utils.square_area(utils.inch_to_m(12)*proto_scale)
        # dominos small size pizza or meal plate
        # height: 5 cm
        # ref : https://www.amazon.in/Microwave-Compartment-Unbreakable-Plastic-Enterprises/dp/B095YMHVHB/ref=sr_1_1?crid=BDLB0C94VSV6&keywords=packing+plates&qid=1640257747&sprefix=packing+plate%2Caps%2C202&sr=8-1
        self.payload_area = utils.square_area(utils.inch_to_m(10))


        # tip to tip excluding fuselage
        self.wing_span = 1.2 * proto_scale
        self.aspect_ratio = 4.0
        self.wing_chord = self.wing_span/self.aspect_ratio
        # self.wing_chord = 0.39

        self.num_vertical_props = 4
        self.num_horizontal_props = 1
        # zero value init
        self.battery_mass = 0
        self.cruise_speed = utils.kmph_to_mps(100) * proto_scale
        self.climb_speed = 4 * proto_scale
        self.land_speed = 4 * proto_scale

        self.horizontal_prop_radius = utils.inch_to_m(10)/2#utils.inch_to_m(12)/2
        self.vertical_prop_radius = self.horizontal_prop_radius
        self.vertical_prop_thrust_max=1.2*self.gravity#2.5*self.gravity
        # todo: verify this, find aerofoil which gives this for selected cruise speed
        self.lift_to_drag = 10

        # ref: table 1: https://downloads.hindawi.com/journals/ijae/2016/3570581.pdf
        self.eff_battery_discharge = 0.95
        self.eff_esc = 0.95
        self.eff_motor = 0.9
        # ref: table 1 - https://downloads.hindawi.com/journals/ijae/2016/3570581.pdf
        # for li-po battery
        self.battery_specific_energy = 150*3600 # joules/kg
        # for li-ion battery
        # self.battery_specific_energy = 240*3600 # joules/kg


        self.kappa=1.15

        self.cost_per_joule = 8.5/(3.6*1e6)

    def wings_area_wet_top(self):
        return self.wing_span*self.wing_chord

    def area_wet_top(self):
        # margin between components, prop area is excluded
        structure_area = self.payload_area
        fuselage_area = structure_area + self.payload_area
        area_top = fuselage_area + self.wings_area_wet_top()
        return area_top

    def get_drag(self, speed, area_wet):
        return (self.atmospheric_density * speed**2 * area_wet)/2

    def eff_transmission(self):
        return self.eff_battery_discharge*self.eff_esc*self.eff_motor

    def prop_area(self, prop_radius):
        return utils.circle_area(prop_radius)

    def motor_mass(self):
        # esc+prop+motor
        # ref: https://www.amazon.in/Invento-ESC1045-Brushless-2200Kv-Propeller/dp/B01MZFNLEA
        return 0.1
        # ref: https://hobbyking.com/en_us/turnigy-aerodrive-sk3-3542-800kv-brushless-outrunner-motor.html?queryID=ec887b1d56d640905cf20ed6747fdab6&objectID=17214&indexName=hbk_live_products_analytics&___store=en_us
        # return 0.142

    def structural_mass(self):
        return self.estimate_wingcopter_structure_mass() * proto_scale**3 # volume scaling

    def set_battery_mass(self, battery_mass):
        self.battery_mass = battery_mass

    def total_mass(self):
        total_mass = self.battery_mass\
            + self.structural_mass()\
            + (self.num_horizontal_props+self.num_vertical_props)*self.motor_mass()\
            + self.payload_mass
        return total_mass

    def estimate_wingcopter_structure_mass(self):
        # wingcopter empty weight estimate
        # http://geozone.ch/images/download/Wingcopter/Wingcopter%20Geozone%20eng.pdf
        # https://www.dvpt.de/uploads_extern/dvpt/2017/pdl/wingcopter.pdf
        take_off_mass = 9
        payload_mass = 2

        # https://www.getfpv.com/tattu-28000mah-22-2v-25c-6s1p-lipo-battery-pack.html
        # tattu- less is more
        # capacity = 28 #Ah
        # battery_capacity = 621.6 #Wh
        # voltage = 22.2
        battery_mass = 3.413

        empty_mass = take_off_mass - (payload_mass + battery_mass)

        # motor https://www.kdedirect.com/products/kde2814xf-515
        motor_mass = 4*0.125
        structural_mass = empty_mass - motor_mass  # tilt motors not considered
        return structural_mass
