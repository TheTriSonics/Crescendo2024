threshold = 1.7  # degrees
yaw = None  # This means we've lost all targets
if yaw is None or abs(yaw) >= threshold:
    rot = 'pid value'
else:
    rot = 0

print(f'rotation: {rot}')
