from shapely.geometry import shape, MultiPolygon
import fiona
#import logging
#import sys
#logging.basicConfig(stream=sys.stdout, level=logging.INFO)

shp_border_region = MultiPolygon([shape(pol['geometry']) for pol in fiona.open('data/border_region.shp', 'r')])
#shp_nsw_northern = MultiPolygon([shape(pol['geometry']) for pol in fiona.open('data/nsw_northern.shp', 'r')])
shp_qld_southeast = MultiPolygon([shape(pol['geometry']) for pol in fiona.open('data/qld_southeast.shp', 'r')])

def auth(service, layers=[], environ=None, **kw):
    if service != 'wms.map' or not layers[0].endswith('_border'):
        return {'authorized':'full'}
    else:
        return {
            'authorized':'partial',
            'layers':
            {
                'nsw_topo3_border': {
                    'map': True,
                    'limited_to':
                    {
                        'geometry': shp_border_region.wkt,
                        'srs': 'EPSG:28356'
                    }
                },
                'qld_topo_border':
                {
                    'map': True,
                    'limited_to':
                    {
                        'geometry': shp_qld_southeast.wkt,
                        'srs': 'EPSG:28356'
                    }
                }
            }
        }

from mapproxy.wsgiapp import make_wsgi_app
_application = make_wsgi_app(r'/home/yellow/mapproxy/aus_src.yaml')

def application(environ, start_response):
    environ['mapproxy.authorize'] = auth
    return _application(environ, start_response)
