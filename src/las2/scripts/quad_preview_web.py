#!/usr/bin/env python3
"""Serve bounded latest JPEG snapshots; never open the camera or run inference."""
import argparse
import html
import json
import os
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import time
from urllib.parse import urlsplit

PAGE = '''<!doctype html><html lang="zh"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ROBOT 四向 RGB / 深度预览</title><style>
body{background:#101820;color:#e5eff5;font:16px sans-serif;margin:24px}
main{display:grid;grid-template-columns:repeat(auto-fit,minmax(420px,1fr));gap:16px}
article{background:#1c2c38;padding:12px;border-radius:12px}img{width:100%}
small{color:#a4bdcc}.stale{color:#ffb66b}h1{font-size:24px}
</style><h1>四向 RGB / 深度预览</h1>
<p>每组左侧：配准 RGB；右侧：深度（近红、远蓝，黑色无效）。
深度显示范围：0–MAX_DEPTH 米。网页预览 FPS_LIMIT Hz，与推理频率独立。</p>
<p>配置：CONFIG_FILE<br>底部裁剪：CROP_SUMMARY（修改后重启生效）</p>
<main id="views"></main><script>
const names=[['front','前 0/1'],['right','右 1/2'],['rear','后 2/3'],['left','左 3/0']];
for(const [key,label] of names){
 const card=document.createElement('article');
 card.innerHTML=`<h2>${label}</h2><small>等待数据</small><img alt="${label} RGB 与深度">`;
 document.querySelector('#views').append(card);
 const img=card.querySelector('img'), status=card.querySelector('small');
 let lastFrame='', frames=0, epoch=performance.now(), rate='';
 async function refresh(){const started=performance.now();try{
 const response=await fetch('/'+key+'.jpg?t='+Date.now(),{cache:'no-store'});
 if(!response.ok)throw Error('等待数据或链路已停止');
 const blob=await response.blob(), url=URL.createObjectURL(blob);
 const previous=img.src;img.src=url;if(previous.startsWith('blob:'))URL.revokeObjectURL(previous);
 const stamp=response.headers.get('X-Frame-Timestamp');
 if(stamp && stamp!==lastFrame){frames++;lastFrame=stamp;}
 const elapsed=performance.now()-epoch;
 if(elapsed>=2000){rate=(frames*1000/elapsed).toFixed(1)+' Hz';frames=0;epoch=performance.now();}
 status.textContent='实时更新 '+rate;status.className='';
 }catch(e){status.textContent=e.message;status.className='stale';}
 setTimeout(refresh,Math.max(0,1000/FPS_LIMIT-(performance.now()-started)));}refresh();
}
</script></html>'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', default='/dev/shm/quad_preview')
    parser.add_argument('--bind', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8088)
    parser.add_argument('--fps', type=float, default=10.0)
    parser.add_argument('--max-depth', type=float, default=10.0)
    parser.add_argument('--config-file', default='')
    parser.add_argument('--crop-ratios', type=float, nargs=4, default=[0.0]*4)
    args = parser.parse_args(sys.argv[1:sys.argv.index('--ros-args')]
                             if '--ros-args' in sys.argv else None)
    if not 0 < args.fps <= 30 or not args.max_depth > 0:
        parser.error('invalid FPS/depth range')
    page = PAGE.replace('MAX_DEPTH', str(args.max_depth)).replace(
        'FPS_LIMIT', str(args.fps)).replace('CONFIG_FILE', html.escape(args.config_file)).replace(
            'CROP_SUMMARY', ' / '.join(f'{name} {value*100:g}%' for name, value in
                                     zip(('前', '右', '后', '左'), args.crop_ratios))).encode()
    effective = json.dumps({'config_file': args.config_file, 'crop_bottom_percent':
                           dict(zip(('front', 'right', 'rear', 'left'),
                                    [v*100 for v in args.crop_ratios]))}).encode()
    root = Path(args.directory)

    class Handler(BaseHTTPRequestHandler):
        def setup(self):
            super().setup()
            self.connection.settimeout(2)

        def do_GET(self):
            route = urlsplit(self.path).path
            content_type = 'text/html; charset=utf-8'
            frame_stamp = None
            if route == '/':
                data = page
            elif route == '/config.json':
                data = effective
                content_type = 'application/json'
            elif route in ('/front.jpg', '/right.jpg', '/rear.jpg', '/left.jpg'):
                try:
                    with (root / route[1:]).open('rb') as stream:
                        stat = os.fstat(stream.fileno())
                        frame_stamp = stat.st_mtime_ns
                        age = time.time() - stat.st_mtime
                        if age > max(3, 3 / args.fps):
                            raise OSError('stale frame')
                        data = stream.read()
                    content_type = 'image/jpeg'
                except OSError:
                    self.send_error(503, 'Waiting for live frame')
                    return
            else:
                self.send_error(404)
                return
            self.send_response(200)
            if frame_stamp is not None:
                self.send_header('X-Frame-Timestamp', str(frame_stamp))
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)

        def log_message(self, *_args):
            pass

    with ThreadingHTTPServer((args.bind, args.port), Handler) as server:
        print(f'Quad preview listening on {args.bind}:{args.port}', flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
