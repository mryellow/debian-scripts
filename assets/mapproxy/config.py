from shapely.geometry import shape
from shapely.geometry import MultiPolygon
import fiona as fio

shp_border_region = MultiPolygon([shape(pol['geometry']) for pol in fio.open('data/border_region.shp', 'r')])
#shp_nsw_border = MultiPolygon([shape(pol['geometry']) for pol in fio.open('data/nsw_northern.shp', 'r')])
shp_qld_border = MultiPolygon([shape(pol['geometry']) for pol in fio.open('data/qld_southeast.shp', 'r')])


def auth(service, layers=[], environ=None, **kw):
    if service != 'wms.map':
        return {'authorized':'full'}
    else:
        return {
            'authorized':'partial',
            'layers':
            {
                'nsw_topo3': {
                    'map': True,
                    'limited_to':
                    {
                        'geometry': shp_border_region.wkt,
                        'srs': 'EPSG:28356'
                    }
                },
                'qld_topo':
                {
                    'map': True,
                    'limited_to':
                    {
                        'geometry': shp_qld_border.wkt,
                        'srs': 'EPSG:28356'
                    }
                }
            }
        }

from mapproxy.wsgiapp import make_wsgi_app
_application = make_wsgi_app(r'/home/yellow/mapproxy/aus_src.yaml')

def application(environ, start_response):
    #environ['mapproxy.authorize'] = auth
    return _application(environ, start_response)
