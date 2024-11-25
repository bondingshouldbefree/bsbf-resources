Add bsbf_resources feed to the bottom of `feeds.conf.default`:
```
src-git bsbf_resources https://github.com/bondingshouldbefree/bsbf-resources.git
```

Refresh feeds and install bsbf_resources feed:
```
./scripts/feeds update && ./scripts/feeds install -a
```
