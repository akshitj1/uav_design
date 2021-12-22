from numpy.lib.index_tricks import AxisConcatenator
import params
import numpy as np
import utils
import math

mission_params = params.MissionParams()
uav_params = params.UavParams(mission_params)


def estimate_battery_mass():
    l=0.
    u=2.0
    eps=0.01
    while True:
        assumed_battery_mass=(l+u)/2
        # iteratively set and compare simulated battery mass
        uav_params.set_battery_mass(assumed_battery_mass)
        print('max climb speed: {:0.1f}'.format(max_climb_speed()))
        print('takeoff mass: {:0.1f} kg'.format(uav_params.total_mass()))
        mission_energy = get_mission_energy()
        req_battery_mass = mission_energy/uav_params.battery_specific_energy
        print('mission energy: {:0.1f} kilo joules\nbattery mass assumed: {:0.1f}\nbattery mass consumed: {:0.1f}'.format(
            mission_energy/1e3,
            uav_params.battery_mass,
            req_battery_mass))
        print('mission electricity cost: {:0.2f} rs'.format(mission_energy*uav_params.cost_per_joule))
        print('-'*20)
        if req_battery_mass<assumed_battery_mass:
            u=assumed_battery_mass
        else:
            l=assumed_battery_mass
        if u-l<eps:
            break

    assert(get_cruise_thrust()<uav_params.num_horizontal_props*uav_params.vertical_prop_thrust_max)
    print('cruise thrust: {:0.2f} kgs'.format(get_cruise_thrust()/uav_params.gravity))

def wing_sizing():
    # ref: table 4 - https://www.sciencedirect.com/science/article/pii/S2215098619316489
    zero_lift_drag_coeff=0.0447
    oswald_eff_factor=0.8
    # todo: AR too low
    aspect_ratio=uav_params.wing_span/uav_params.wing_chord
    # oswald_eff_factor=1.78*(1-0.045*pow(aspect_ratio, 0.68))-0.64
    print('oswald eff. factor: {:0.1f}'.format(oswald_eff_factor))
    K=1/(math.pi*oswald_eff_factor*aspect_ratio)
    lift_coeff_cruise = math.sqrt(zero_lift_drag_coeff/K)
    wing_suraface_area = (2*get_takeoff_weight())/(lift_coeff_cruise*uav_params.atmospheric_density*math.pow(uav_params.cruise_speed,2))
    wing_span = math.sqrt(wing_suraface_area*aspect_ratio)
    # todo: underestimating wing size, not in par with historic data
    print('wing span: {:0.2f} m'.format(wing_span))


def get_takeoff_weight():
    return uav_params.total_mass() * uav_params.gravity


def get_climb_thrust():
    # todo: correct this
    assert(uav_params.climb_speed<max_climb_speed())
    induced_drag = uav_params.get_drag(
        speed=uav_params.climb_speed, area_wet=uav_params.area_wet_top())
    thrust_req = get_takeoff_weight() + induced_drag
    return thrust_req

def max_climb_speed():
    residual_thrust = uav_params.num_vertical_props*uav_params.vertical_prop_thrust_max - get_takeoff_weight()
    max_climb_speed=math.sqrt((2*residual_thrust)/(2*uav_params.atmospheric_density*uav_params.area_wet_top()))
    return max_climb_speed

def get_climb_pow():
    thrust_req = get_climb_thrust()
    climb_power_drain = get_propeller_power_drain(
        thrust=thrust_req,
        prop_radius=uav_params.vertical_prop_radius,
        num_props=uav_params.num_vertical_props,
        axial_speed=uav_params.climb_speed)
    return climb_power_drain


def time_taken(speed, distance):
    return distance/speed


def get_climb_time():
    climb_time = time_taken(speed=uav_params.climb_speed,
                            distance=mission_params.cruise_height)
    return climb_time


def get_climb_energy():
    power_drain = get_climb_pow()
    energy_req_climb = get_climb_time() * power_drain
    return energy_req_climb


def get_land_pow():
    # ref: sec 2.2.5 https://downloads.hindawi.com/journals/ijae/2016/3570581.pdf
    hover_thrust_props = get_takeoff_weight()
    hover_thrust_prop = hover_thrust_props/uav_params.num_vertical_props
    vel_hover_induced = math.sqrt(
        hover_thrust_prop
        / (2*uav_params.atmospheric_density*uav_params.prop_area(uav_params.vertical_prop_radius))
    )
    # low speed axial descent condn.
    assert(uav_params.land_speed<2*vel_hover_induced)
    # todo: check sign
    x = -uav_params.land_speed/vel_hover_induced
    vel_land_induced = np.dot(
        np.array([uav_params.kappa, -1.125, -1.372, -1.718, -0.655]),
        np.array([math.pow(x, exp) for exp in range(5)])
    ) * vel_hover_induced
    
    induced_drag = uav_params.get_drag(
        speed=uav_params.land_speed, area_wet=uav_params.area_wet_top())
    land_thrust = get_takeoff_weight() - induced_drag
    assert(land_thrust > 0)
    land_thrust_prop = land_thrust/uav_params.num_vertical_props
    # todo: check sign
    # todo: original paper on total thrust instead this!! verify
    land_power_prop = uav_params.kappa * land_thrust_prop * \
        (vel_land_induced-uav_params.land_speed)
    assert(vel_land_induced-uav_params.land_speed>0)
    land_power_props = uav_params.num_vertical_props * land_power_prop
    land_power_drain = transmission_power(land_power_props)
    return land_power_drain


def get_land_time():
    return time_taken(speed=uav_params.land_speed, distance=mission_params.cruise_height)


def get_land_energy():
    power_drain = get_land_pow()
    energy_land = get_land_time()*get_land_pow()
    return energy_land


def get_cruise_thrust():
    # todo: refine this
    thrust_req = get_takeoff_weight()/uav_params.lift_to_drag
    return thrust_req


def transmission_power(power_req):
    # actual power req. from batteries accounting losses
    return power_req/uav_params.eff_transmission()


def get_propeller_power_drain(thrust, prop_radius, num_props, axial_speed):
    thrust_per_prop = thrust/num_props
    momentum_theory_power = (0.5 * uav_params.kappa * thrust_per_prop * axial_speed) \
        * (1
            + math.sqrt(
                1+(2*thrust_per_prop)/(uav_params.atmospheric_density *
                                       uav_params.prop_area(prop_radius)*axial_speed**2)
            )
           )
    power_drain_prop = transmission_power(momentum_theory_power)
    power_drain_props = num_props*power_drain_prop
    return power_drain_props


def get_cruise_pow():
    thrust_req = get_cruise_thrust()
    power_drain = get_propeller_power_drain(
        thrust_req,
        uav_params.horizontal_prop_radius,
        uav_params.num_horizontal_props,
        axial_speed=uav_params.cruise_speed
    )
    return power_drain


def get_cruise_time():
    return time_taken(speed=uav_params.cruise_speed, distance=mission_params.cruise_distance)


def energy_consumed(power, time):
    return power*time


def get_cruise_energy():
    energy_req_cruise = energy_consumed(get_cruise_pow(), get_cruise_time())
    return energy_req_cruise


def get_all_vflight_energy():
    assert(get_climb_energy()>get_land_energy())
    vflight_energy =\
        mission_params.num_climbs*get_climb_energy()\
        + mission_params.num_lands*get_land_energy()
    return vflight_energy


def get_mission_energy():
    print('cruise energy: {:0.1f} kJ\tvflight energies: {:0.1f} kJ'.format(
        get_cruise_energy()/1e3, get_all_vflight_energy()/1e3))
    mission_energy = get_all_vflight_energy()+get_cruise_energy()
    return mission_energy


def simulate_food_delivery():
    estimate_battery_mass()
    wing_sizing()
    print("nearest charge point: {:0.1f} kms".format(
        np.round(mission_params.charge_point_distance/1e3, 1)))


if __name__ == '__main__':
    simulate_food_delivery()
