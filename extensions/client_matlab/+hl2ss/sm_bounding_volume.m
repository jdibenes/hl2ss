
classdef sm_bounding_volume < handle
properties
    volumes
end

methods
    function self = sm_bounding_volume()
        self.volumes = struct([]);
    end
    
    function add(self, type, center, extents, near, far, right, left, top, bottom, orientation, radius)
        volume = struct();
        volume.type        = type;
        volume.center      = center;
        volume.extents     = extents;
        volume.near        = near;
        volume.far         = far;
        volume.right       = right;
        volume.left        = left;
        volume.top         = top;
        volume.bottom      = bottom;
        volume.orientation = orientation;
        volume.radius      = radius;
        
        self.volumes = [self.volumes; volume];
    end
    
    function add_box(self, center, extents)
        self.add(hl2ss.sm_volume_type.Box, single(center), single(extents), [], [], [], [], [], [], [], []);
    end
    
    function add_frustum(self, near, far, right, left, top, bottom)
        self.add(hl2ss.sm_volume_type.Frustum, [], [], single(near), single(far), single(right), single(left), single(top), single(bottom), [], []);
    end
    
    function add_oriented_box(self, center, extents, orientation)
        self.add(hl2ss.sm_volume_type.OrientedBox, single(center), single(extents), [], [], [], [], [], [], single(orientation), []); 
    end
    
    function add_sphere(self, center, radius)
        self.add(hl2ss.sm_volume_type.Sphere, single(center), [], [], [], [], [], [], [], [], single(radius));
    end
end
end
