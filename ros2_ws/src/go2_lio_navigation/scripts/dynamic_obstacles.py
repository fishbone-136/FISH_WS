"""Bounded motion evidence for compact obstacles; never a blind-space TTL."""
import math
import numpy as np
from scipy.ndimage import label
from scipy.spatial import cKDTree
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import connected_components


def cells_in_bounds(memory, lo, hi, resolution):
    """Probe a compact track's grid window instead of rescanning the map."""
    lower=np.floor(np.asarray(lo)/resolution).astype(int)
    upper=np.floor(np.asarray(hi)/resolution).astype(int)
    area=int(np.prod(upper-lower+1))
    if area<len(memory):
        def present():
            for x in range(int(lower[0]),int(upper[0])+1):
                for y in range(int(lower[1]),int(upper[1])+1):
                    value=memory.get((x,y))
                    if value is not None:yield (x,y),value
        candidates=present()
    else:candidates=memory.items()
    return ((k,v) for k,v in candidates if lo[0]<=v[0]<=hi[0] and lo[1]<=v[1]<=hi[1])


class DynamicObstacles:
    def __init__(self,resolution=.05):
        self.resolution=resolution
        self.tracks={}
        self.dynamic_cells={}
        self.first_seen={}
        self.support_history={}
        self.static_confirmed={}
        self.untracked_landmarks={}
        self.tracked_seen=set()
        self.next_id=1
        self.last_stamp=-math.inf
        self.confirmed_tracks=0

    def observe(self,points,pose,stamp,memory):
        if stamp-self.last_stamp<.05:return
        self.last_stamp=stamp
        points=np.asarray(points,dtype=float).reshape(-1,3)
        points=points[np.isfinite(points).all(axis=1)]
        near=np.sum((points[:,:2]-pose[:2])**2,axis=1)<=5.5**2
        points=points[near]
        self.tracks={k:v for k,v in self.tracks.items() if stamp-v['stamp']<2.}
        if not len(points):return
        previous_dynamic=self.dynamic_cells.copy()
        keys=np.floor(points[:,:2]/self.resolution).astype(np.int64)
        key_tuples=list(map(tuple,keys.tolist()))
        all_keys=set(key_tuples)
        for k in all_keys:
            self.first_seen.setdefault(k,stamp)
            first,last,count=self.support_history.get(k,(stamp,stamp,0))
            if stamp-last>.6:first,count=stamp,0
            count+=1
            self.support_history[k]=(first,stamp,count)
            if count>=6 and stamp-first>=1.:
                self.static_confirmed.setdefault(k,stamp)
        coarse=np.floor(points[:,:2]/.15).astype(int)
        lower=coarse.min(axis=0);indices=coarse-lower
        size=indices.max(axis=0)+1
        if np.any(size>90):return
        grid=np.zeros(tuple(size),dtype=bool);grid[indices[:,0],indices[:,1]]=True
        labels,_=label(grid,structure=np.ones((3,3),dtype=bool))
        point_labels=labels[indices[:,0],indices[:,1]]
        clusters=[]
        def compact(mask):
            cloud=points[mask]
            # A close leg may contain only a handful of returns. Require
            # measured width AND height plus the existing multi-frame motion
            # confirmation; a single thin static pole still cannot qualify.
            if len(cloud)<4:return None
            extent=np.ptp(cloud[:,:2],axis=0)
            if not .10<=max(extent)<=1.2 or np.ptp(cloud[:,2])<.12:return None
            cells=set(map(tuple,keys[mask].tolist()))
            center=.5*(cloud[:,:2].min(axis=0)+cloud[:,:2].max(axis=0))
            return center,extent,cells

        recent=np.fromiter((stamp-self.first_seen[k]<3. or k in self.dynamic_cells
                            for k in key_tuples),bool,count=len(keys))
        order = np.argsort(point_labels, kind='stable')
        splits = np.flatnonzero(np.diff(point_labels[order])) + 1
        for group_indices in np.split(order, splits):
            if len(group_indices) < 4:
                continue
            # A person passing next to a box/wall can merge with it in XY.
            # Split recently appearing geometry away from established static
            # returns before tracking; otherwise changing combined dimensions
            # repeatedly restart the track and leave untagged body trails.
            young = group_indices[recent[group_indices]]
            if len(group_indices) - len(young) >= 8 and len(young) >= 8:
                fg=np.zeros(tuple(size),dtype=bool)
                fg[indices[young,0],indices[young,1]]=True
                groups,_=label(fg,structure=np.ones((3,3),dtype=bool))
                foreground=groups[indices[young,0],indices[young,1]]
                separated=[compact(young[foreground==v]) for v in np.unique(foreground)]
                separated=[v for v in separated if v is not None]
                if separated:
                    clusters.extend(separated)
                    continue
            # A single thin leg and long walls never become moving objects.
            cluster=compact(group_indices)
            if cluster is not None:clusters.append(cluster)
        used=set();claimed=set()
        for center,extent,cells in clusters:
            best=None;distance=math.inf
            for identity,track in self.tracks.items():
                if identity in used:continue
                dt=stamp-track['stamp'];shift=np.linalg.norm(center-track['center'])
                shape=np.max(np.abs(extent-track['extent']))
                if dt>0. and shift<min(1.2,2.*dt+.08) and shape<.20 and shift<distance:
                    best=identity;distance=shift
            if best is None:
                if len(self.tracks)>=24:continue
                best=self.next_id;self.next_id+=1
                self.tracks[best]={'first':center.copy(),'birth':stamp,'pose':tuple(pose),
                    'center':center,'extent':extent,'stamp':stamp,'updates':0,
                    'cells':{},'protected':set(),'dynamic':False,'trail':[]}
            track=self.tracks[best];used.add(best);claimed.update(cells)
            self.tracked_seen.update(cells)
            # Preserve cells already established before this object arrived.
            track['protected'].update(k for k in cells if k in memory and
                (self.static_confirmed.get(k,math.inf)<track['birth']-.50 or
                 self.untracked_landmarks.get(k,math.inf)<track['birth']))
            track['cells'].update({k:stamp for k in cells})
            track['cells']={k:t for k,t in track['cells'].items() if stamp-t<4.}
            track['updates']+=1
            track['trail'].append((stamp,center.copy(),pose[2]))
            track['trail']=[v for v in track['trail'] if stamp-v[0]<=1.6][-24:]
            trail=track['trail'];duration=stamp-trail[0][0]
            ego_yaw=abs(math.remainder(pose[2]-trail[0][2],2*math.pi))
            moved=np.linalg.norm(center-trail[0][1])
            travel=sum(np.linalg.norm(b[1]-a[1]) for a,b in zip(trail,trail[1:]))
            if (not track['dynamic'] and ego_yaw<=.20 and .18<=duration<=2.0 and
                    len(trail)>=4 and moved>=.20 and moved>=.80*travel):
                track['dynamic']=True;self.confirmed_tracks+=1
            if track['dynamic']:
                # Include fragmented arm/leg returns and the first few frames
                # before the complete body cluster became trackable. Protect
                # independently established static geometry in this corridor.
                centers=np.asarray([v[1] for v in trail])
                margin=.5*max(max(extent),max(track['extent']))+.10
                lo=centers.min(axis=0)-margin;hi=centers.max(axis=0)+margin
                for k,v in cells_in_bounds(memory,lo,hi,self.resolution):
                    if (self.first_seen.get(k,v[2])>=track['birth']-2. and
                            self.static_confirmed.get(k,math.inf)>=track['birth']-.50 and
                            self.untracked_landmarks.get(k,math.inf)>=track['birth']):
                        track['cells'][k]=stamp
                for k in track['cells'].keys()-track['protected']:
                    self.dynamic_cells[k]=best
            else:
                for k in cells:
                    if self.dynamic_cells.get(k)!=best:self.dynamic_cells.pop(k,None)
            track.update(center=center,extent=extent,stamp=stamp)
        # Fragmentation can temporarily fail compact() or create a new track
        # with too little motion history. Preserve an existing association
        # only to a recently confirmed moving owner, at cells it already
        # owned. This does not recruit a new thin leg/static neighbour.
        fragments=set()
        for k in all_keys:
            owner=previous_dynamic.get(k)
            track=self.tracks.get(owner)
            if (track is not None and track['dynamic'] and
                    0.<=stamp-track['stamp']<=.60 and k not in track['protected'] and
                    self.dynamic_cells.get(k,owner)==owner):
                self.dynamic_cells[k]=owner;fragments.add(k)
        # Independently untracked evidence (including a new leg) owns its
        # cells immediately; confirmed moving fragments retain their owner.
        for k in all_keys-claimed:
            if k in fragments:continue
            self.dynamic_cells.pop(k,None)
            if k not in self.tracked_seen:self.untracked_landmarks.setdefault(k,stamp)

    def prune(self,memory):
        self.dynamic_cells={k:v for k,v in self.dynamic_cells.items() if k in memory}
        self.first_seen={k:v for k,v in self.first_seen.items() if k in memory}
        self.support_history={k:v for k,v in self.support_history.items() if k in memory}
        self.static_confirmed={k:v for k,v in self.static_confirmed.items() if k in memory}
        self.untracked_landmarks={k:v for k,v in self.untracked_landmarks.items() if k in memory}
        self.tracked_seen.intersection_update(memory)

    def forget_cells(self, keys):
        for key in keys:
            self.dynamic_cells.pop(key, None)
            self.first_seen.pop(key, None)
            self.support_history.pop(key, None)
            self.static_confirmed.pop(key, None)
            self.untracked_landmarks.pop(key, None)
        self.tracked_seen.difference_update(keys)

    def flags(self,keys):
        return np.fromiter((k in self.dynamic_cells for k in keys),bool,count=len(keys))

    def transient_flags(self,keys,memory):
        """Group short-lived fragments; never expire geometry or a sparse landmark.

        Flags permit repeated measured-ground visibility with the normal
        eight-observation rule, rather than declaring these objects moving.
        """
        result=np.zeros(len(keys),dtype=bool)
        candidates=[]
        for i,key in enumerate(keys):
            history=self.support_history.get(key)
            # Near HAP returns can hit a passing fragment six times before
            # the one-second static confirmation. Packet count is not proof
            # of a landmark: keep grouped fragments eligible until confirmed.
            if (key not in self.static_confirmed and history is not None and
                    history[1]-history[0]<=1.5 and key in memory):
                candidates.append(i)
        if len(candidates)<4:return result
        xy=np.asarray([memory[keys[i]][:2] for i in candidates])
        # Use measured distances: float32 returns either side of zero can
        # acquire nonadjacent integer bins despite being only 10 cm apart.
        pairs=cKDTree(xy).query_pairs(.12,output_type='ndarray')
        graph=csr_matrix((np.ones(len(pairs),dtype=bool),(pairs[:,0],pairs[:,1])),
                         shape=(len(xy),len(xy)))
        components, identities = connected_components(graph, directed=False)
        lower = np.full((components, 2), np.inf)
        upper = np.full((components, 2), -np.inf)
        np.minimum.at(lower, identities, xy)
        np.maximum.at(upper, identities, xy)
        wide = np.max(upper - lower, axis=1) >= .15
        grouped = wide & (np.bincount(identities, minlength=components) >= 4)
        result[candidates] = grouped[identities]
        return result
