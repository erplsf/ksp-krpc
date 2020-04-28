import krpc
import time

conn = krpc.connect(name='Launcher')
vessel = conn.space_center.active_vessel
flight = vessel.flight()
kerbin_flight = vessel.flight(vessel.orbit.body.reference_frame)


def pitch_func(altitude):
    # magic constants calculated from Desmos
    a = -0.258652
    b = 0.530676
    k = -92.9603
    pitch_angle = a * (altitude ** b) - k

    if altitude < 0:
        return float(90)
    elif altitude > 80_000:
        return float(0)
    else:
        return pitch_angle


vessel.control.sas = False
vessel.control.rcs = False
vessel.control.throttle = 1.0

vessel.control.activate_next_stage()
vessel.auto_pilot.target_heading = 90
vessel.auto_pilot.target_pitch = 90
vessel.auto_pilot.engage()

target_apoapsis = 90_000
target_apo_reached = False

s3_resources_amount = conn.add_stream(vessel.resources_in_decouple_stage(stage=3, cumulative=False).amount, 'LiquidFuel')
s3_separated = False

while True:
    # "gravity" turn
    target_pitch = pitch_func(flight.mean_altitude)
    current_pitch = flight.pitch
    if (1 - (target_pitch / current_pitch)) > 0.01:
        vessel.auto_pilot.target_pitch = target_pitch
    # speed = kerbin_flight.speed
    # print(f"pitch diff: {current_pitch - target_pitch}")

    if not s3_separated and s3_resources_amount() < 0.1:
        vessel.control.activate_next_stage()
        s3_separated = True

    # apoapsis/throttle control
    apoapsis = vessel.orbit.apoapsis_altitude
    # print(f"current_apo: {apoapsis}, target: {target_apoapsis}")
    if not target_apo_reached:
        if (1 - (apoapsis / target_apoapsis)) < 0.05:
            current_throttle = vessel.control.throttle
            vessel.control.throttle = max(current_throttle * 0.9, 0.1)
        elif (1 - (apoapsis / target_apoapsis)) < 0.01:
            vessel.control.throttle = 0
            target_apo_reached = True
    if (1 - (apoapsis / target_apoapsis)) < 0.01:
        print("Program finished, target apoapsis reached.")
        vessel.control.throttle = 0
        vessel.auto_pilot.disengage()
        break
    time.sleep(1)
