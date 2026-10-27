#!/usr/bin/bash
if [ "$0" = "-bash" ]; then
    _v2_both_completion() {
	local v2cmd
	v2cmd=(enable disable go_zero joint reset stop read_joint)
	local cur prev opts
	cur="${COMP_WORDS[COMP_CWORD]}"
	prev="${COMP_WORDS[COMP_CWORD-1]}"
	if [ ${COMP_CWORD} -eq 1 ]; then
	    # v2cmd 配列の内容を候補として返す
	    COMPREPLY=( $(compgen -W "${v2cmd[*]}" -- ${cur}) )
	    return 0
	else
	    # 第2引数以降は通常のファイル名補完を行う
	    compopt -o filenames
	    COMPREPLY=( $(compgen -f -- ${cur}) )
	    return 0
	fi
    }
    complete -F _v2_both_completion v2_both.sh
    complete -F _v2_both_completion ./v2_both.sh
    complete -F _v2_both_completion ./V2Cmd/v2_both.sh
    return
fi
cd `dirname "$0"`
python ./piper_v2_cmd.py -r "$@"
python ./piper_v2_cmd.py -l "$@"
