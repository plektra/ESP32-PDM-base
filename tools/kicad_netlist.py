#!/usr/bin/env python3
"""Minimal KiCad 8 schematic netlister / connectivity checker (no KiCad needed).

Usage: python3 tools/kicad_netlist.py ESP32-PDM-BASE.kicad_sch

Walks the sheet hierarchy, resolves wires, junctions, local/global/hierarchical
labels, sheet pins and power symbols, then prints every net with its pins,
followed by warnings (floating labels, unmatched hierarchical labels) and
single-pin nets without a no-connect flag. Buses are not handled.
Not a replacement for KiCad ERC.
"""
import sys, math, re, os
from collections import defaultdict

def tokenize(s):
    i = 0; n = len(s)
    while i < n:
        c = s[i]
        if c in '()':
            yield c; i += 1
        elif c.isspace():
            i += 1
        elif c == '"':
            j = i + 1; buf = []
            while s[j] != '"':
                if s[j] == '\\':
                    buf.append(s[j+1]); j += 2
                else:
                    buf.append(s[j]); j += 1
            yield ('S', ''.join(buf)); i = j + 1
        else:
            j = i
            while j < n and not s[j].isspace() and s[j] not in '()':
                j += 1
            yield s[i:j]; i = j

def parse(s):
    stack = [[]]
    for t in tokenize(s):
        if t == '(':
            stack.append([])
        elif t == ')':
            x = stack.pop(); stack[-1].append(x)
        else:
            stack[-1].append(t[1] if isinstance(t, tuple) else t)
    return stack[0][0]

def find(node, key):
    return [c for c in node if isinstance(c, list) and c and c[0] == key]

def first(node, key):
    r = find(node, key); return r[0] if r else None

def prop(node, name):
    for p in find(node, 'property'):
        if p[1] == name: return p[2]

R = lambda v: round(float(v), 2)

def pt(x, y): return (R(x), R(y))

class Sheet:
    def __init__(self, path, name):
        self.file = path; self.name = name
        self.tree = parse(open(path).read())

def lib_pins(libsym_node):
    """returns {unit: [(num,name,type,x,y,hidden)]}, unit 0 = common"""
    pins = defaultdict(list)
    is_power = first(libsym_node, 'power') is not None
    for sub in find(libsym_node, 'symbol'):
        m = re.match(r'.*_(\d+)_(\d+)$', sub[1])
        unit = int(m.group(1))
        for p in find(sub, 'pin'):
            at = first(p, 'at')
            hidden = 'hide' in p or any(isinstance(c, list) and c[0]=='hide' and c[1:]==['yes'] for c in p)
            pins[unit].append((first(p, 'number')[1], first(p, 'name')[1], p[1], float(at[1]), float(at[2]), hidden))
    return pins, is_power

def xform(px, py, at, mirror):
    x, y = px, -py
    rot = int(float(at[3])) if len(at) > 3 else 0
    th = math.radians(rot)
    c, s = round(math.cos(th)), round(math.sin(th))
    x, y = x * c + y * s, -x * s + y * c
    if mirror == 'x': y = -y
    if mirror == 'y': x = -x
    return pt(float(at[1]) + x, float(at[2]) + y)

def on_seg(p, a, b):
    (x, y), (x1, y1), (x2, y2) = p, a, b
    if abs((x2-x1)*(y-y1) - (y2-y1)*(x-x1)) > 1e-3: return False
    return min(x1,x2)-1e-3 <= x <= max(x1,x2)+1e-3 and min(y1,y2)-1e-3 <= y <= max(y1,y2)+1e-3

class UF:
    def __init__(s): s.p = {}
    def f(s, a):
        s.p.setdefault(a, a)
        while s.p[a] != a:
            s.p[a] = s.p[s.p[a]]; a = s.p[a]
        return a
    def u(s, a, b): s.p[s.f(a)] = s.f(b)

uf = UF()
pins_all = []   # (node, sheet, ref, num, name, type, value)
names = defaultdict(set)  # uf node -> names
nc_points = []
warnings = []

def process(sheet, sname):
    t = sheet.tree
    libs = {}
    for ls in find(first(t, 'lib_symbols'), 'symbol'):
        libs[ls[1]] = lib_pins(ls)
    wires = [(pt(*w_[0][1:3]), pt(*w_[1][1:3])) for w_ in (first(w, 'pts')[1:] for w in find(t, 'wire'))]
    for b in find(t, 'bus'):
        warnings.append(f'{sname}: bus present (not handled)')
    junctions = [pt(*first(j, 'at')[1:3]) for j in find(t, 'junction')]
    K = lambda p: (sname, p)
    # wire segments connect at their endpoints
    wire_ends = set()
    for a, b in wires:
        uf.u(K(a), K(b)); wire_ends.add(a); wire_ends.add(b)
    points = []  # all connection points: (p, kind)
    def attach(p, kind):
        hit = False
        for a, b in wires:
            if on_seg(p, a, b):
                uf.u(K(p), K(a)); hit = True
        return hit
    # wire endpoints landing on other wires (T) & junctions
    for p in list(wire_ends) + junctions:
        attach(p, 'w')
    # symbols
    for s in find(t, 'symbol'):
        lib_id = first(s, 'lib_id')[1]
        at = first(s, 'at'); mir = first(s, 'mirror'); mir = mir[1] if mir else None
        unit = int(first(s, 'unit')[1]) if first(s, 'unit') else 1
        ref = prop(s, 'Reference'); val = prop(s, 'Value')
        # instance ref override
        inst = first(s, 'instances')
        if inst:
            for pr in find(inst, 'project'):
                for pa in find(pr, 'path'):
                    r = first(pa, 'reference')
                    if r: ref = r[1]
        pins, is_power = libs[lib_id]
        if first(s, 'lib_name'):
            pins, is_power = libs[first(s, 'lib_name')[1]]
        for (num, name, typ, px, py, hidden) in pins.get(0, []) + pins.get(unit, []):
            p = xform(px, py, at, mir)
            node = K(p)
            uf.f(node)
            hit = attach(p, 'pin')
            if is_power:
                names[node].add(('global', val))
            else:
                pins_all.append(dict(node=node, sheet=sname, ref=ref, num=num, name=name, type=typ, val=val, lib=lib_id, p=p, hit=hit, unit=unit))
    for l in find(t, 'label'):
        p = pt(*first(l, 'at')[1:3])
        if not attach(p, 'l'): warnings.append(f'{sname}: label {l[1]} at {p} not on wire')
        names[K(p)].add(('local:' + sname, l[1]))
    for l in find(t, 'global_label'):
        p = pt(*first(l, 'at')[1:3])
        if not attach(p, 'l'): warnings.append(f'{sname}: global label {l[1]} at {p} not on wire')
        names[K(p)].add(('global', l[1]))
    for l in find(t, 'hierarchical_label'):
        p = pt(*first(l, 'at')[1:3])
        if not attach(p, 'l'): warnings.append(f'{sname}: hier label {l[1]} at {p} not on wire')
        names[K(p)].add(('hier:' + sname, l[1]))
    for n in find(t, 'no_connect'):
        p = pt(*first(n, 'at')[1:3])
        nc_points.append(K(p)); uf.f(K(p)); attach(p, 'nc')
    subs = []
    for sh in find(t, 'sheet'):
        fn = prop(sh, 'Sheetfile'); nm = prop(sh, 'Sheetname')
        for pn in find(sh, 'pin'):
            p = pt(*first(pn, 'at')[1:3])
            if not attach(p, 's'): warnings.append(f'{sname}: sheet pin {nm}/{pn[1]} at {p} not on wire')
            names[K(p)].add(('sheetpin:' + nm, pn[1]))
            uf.f(K(p))
        subs.append((fn, nm))
    return subs

root = sys.argv[1]
d = os.path.dirname(root)
todo = [(root, 'ROOT')]
while todo:
    f, n = todo.pop()
    for fn, nm in process(Sheet(f, n), n):
        todo.append((os.path.join(d, fn), nm))

# merge by names
groups = defaultdict(list)
for node, ns in names.items():
    for kind, nm in ns:
        if kind.startswith('sheetpin:'):
            key = ('hier:' + kind[9:], nm)
        else:
            key = (kind, nm)
        groups[key].append(node)
for key, nodes in groups.items():
    for n in nodes[1:]: uf.u(nodes[0], n)
# hierarchical labels without sheet pins / sheet pins without labels
hk = {k for k in groups if k[0].startswith('hier:')}
for (kind, nm), nodes in groups.items():
    if kind.startswith('hier:'):
        kinds = set()
        for nd in nodes:
            for kk, n2 in names[nd]:
                if n2 == nm: kinds.add(kk.split(':')[0])
        if kinds != {'hier', 'sheetpin'}:
            warnings.append(f'unmatched hierarchical {kind} {nm}: only {kinds}')

nets = defaultdict(lambda: {'pins': [], 'names': set(), 'nc': False})
for p in pins_all:
    nets[uf.f(p['node'])]['pins'].append(p)
for node, ns in names.items():
    for kind, nm in ns: nets[uf.f(node)]['names'].add((kind, nm))
for n in nc_points:
    nets[uf.f(n)]['nc'] = True

def netname(n):
    g = sorted(nm for k, nm in n['names'] if k == 'global')
    if g: return '/'.join(g)
    o = sorted(f'{k.split(":",1)[1]}/{nm}' for k, nm in n['names'] if not k.startswith('sheetpin'))
    if o: return ' = '.join(o)
    if n['pins']: return f"Net-({n['pins'][0]['ref']}-{n['pins'][0]['num']})"
    return '?'

out = []
for root_, n in nets.items():
    if not n['pins'] and not n['names']: continue
    out.append((netname(n), n))
out.sort(key=lambda x: x[0])
for nm, n in out:
    flag = ' [NC]' if n['nc'] else ''
    print(f'NET {nm}{flag}')
    for p in sorted(n['pins'], key=lambda p: (p['ref'], p['num'])):
        print(f"    {p['ref']:6s} {p['num']:>4s} {p['name']:18s} {p['type']:14s} {p['val']}")
print('\n==== WARNINGS ====')
pinpts = {p['node'] for p in pins_all}
for w in warnings:
    m = re.match(r'(.*?): .*label (\S+) at \(([-\d.]+), ([-\d.]+)\)', w)
    if m and (m.group(1), (float(m.group(3)), float(m.group(4)))) in pinpts:
        continue  # label placed directly on a pin is connected
    print(w)
print('\n==== single-pin nets (unconnected pins w/o NC) ====')
for nm, n in out:
    if len(n['pins']) == 1 and not n['nc'] and not any(k == 'global' for k, _ in n['names']):
        p = n['pins'][0]
        print(f"  {p['sheet']}: {p['ref']} pin {p['num']} ({p['name']}, {p['type']}) net={nm}")
    if len(n['pins']) == 0:
        print(f"  net {nm} has no pins")
print('\n==== NC flag on connected nets ====')
for nm, n in out:
    if n['nc'] and len(n['pins']) > 1: print('  ', nm, [(p['ref'], p['num']) for p in n['pins']])

