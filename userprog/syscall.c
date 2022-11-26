#include "userprog/syscall.h"
#include <stdio.h>
#include <syscall-nr.h>
// #include <palloc.h>
#include "threads/palloc.h"
#include "threads/interrupt.h"
#include "threads/thread.h"
#include "threads/loader.h"
#include "userprog/gdt.h"
#include "userprog/process.h"
#include "threads/flags.h"
#include "intrinsic.h"

void syscall_entry (void);
void syscall_handler (struct intr_frame *);

/* System call.
 *
 * Previously system call services was handled by the interrupt handler
 * (e.g. int 0x80 in linux). However, in x86-64, the manufacturer supplies
 * efficient path for requesting the system call, the `syscall` instruction.
 *
 * The syscall instruction works by reading the values from the the Model
 * Specific Register (MSR). For the details, see the manual. */

#define MSR_STAR         0xc0000081 /* Segment selector msr */
#define MSR_LSTAR        0xc0000082 /* Long mode SYSCALL target */
#define MSR_SYSCALL_MASK 0xc0000084 /* Mask for the eflags */

void
syscall_init (void) {
  write_msr (MSR_STAR, ((uint64_t) SEL_UCSEG - 0x10) << 48 |
                           ((uint64_t) SEL_KCSEG) << 32);
  write_msr (MSR_LSTAR, (uint64_t) syscall_entry);

  /* The interrupt service rountine should not serve any interrupts
   * until the syscall_entry swaps the userland stack to the kernel
   * mode stack. Therefore, we masked the FLAG_FL. */
  write_msr (MSR_SYSCALL_MASK,
             FLAG_IF | FLAG_TF | FLAG_DF | FLAG_IOPL | FLAG_AC | FLAG_NT);
}

static void sys_halt (void);
static void sys_exit (int status);
static tid_t sys_fork (const char *thread_name, struct intr_frame *f);
static int sys_exec (const char *file);
static int sys_wait (pid_t);
static bool sys_create (const char *file, unsigned initial_size);
static bool sys_remove (const char *file);
static int sys_open (const char *file);
static int sys_filesize (int fd);
static int sys_read (int fd, void *buffer, unsigned length);
static int sys_write (int fd, const void *buffer, unsigned length);
static void sys_seek (int fd, unsigned position);
static unsigned sys_tell (int fd);
static void sys_close (int fd);

void
__dump_frame (const struct intr_frame *f) {
  printf ("=====================dump======================\n");
  printf ("interrupt frame's address: %lx\n", f);
  printf ("rax %016llx rbx %016llx rcx %016llx rdx %016llx\n", f->R.rax,
          f->R.rbx, f->R.rcx, f->R.rdx);
  printf ("rsp %016llx rbp %016llx rsi %016llx rdi %016llx\n", f->rsp, f->R.rbp,
          f->R.rsi, f->R.rdi);
  printf ("rip %016llx r8  %016llx r9  %016llx r10 %016llx\n", f->rip, f->R.r8,
          f->R.r9, f->R.r10);
  printf ("r11 %016llx r12 %016llx r13 %016llx r14 %016llx\n", f->R.r11,
          f->R.r12, f->R.r13, f->R.r14);
  printf ("r15 %016llx rflags %08llx\n", f->R.r15, f->eflags);
  printf ("es: %04x ds: %04x cs: %04x ss: %04x\n", f->es, f->ds, f->cs, f->ss);
  printf ("=====================dump======================\n");
}

/* The main system call interface */
void
syscall_handler (struct intr_frame *f) {
  // TODO: Your implementation goes here.
  // 1.return 값은 rax에 담자

  // __dump_frame (f);
  // hex_dump (f, f, sizeof (struct intr_frame), true);

  // hex_dump (&f->rsp, &f->rsp, 256, true);

  // printf ("syscall no: %d\n", f->R.rax);
  // hex_dump (f->R.rsi, f->R.rsi, 256, true);
  // printf ("is kernel: %d\n", is_kernel_vaddr (f->R.rsi));
  // printf ("system call!\n");
  switch (f->R.rax) {
  case SYS_HALT:
    sys_halt ();
    break;

  case SYS_EXIT:
    sys_exit (f->R.rdi);
    break;

  case SYS_FORK:
    f->R.rax = sys_fork (f->R.rdi, f);
    break;

  case SYS_EXEC:
    sys_exec (f->R.rdi);
    break;

  case SYS_WAIT:
    f->R.rax = sys_wait (f->R.rdi);
    break;

  case SYS_CREATE:
    break;

  case SYS_REMOVE:
    break;

  case SYS_OPEN:
    break;

  case SYS_FILESIZE:
    break;

  case SYS_READ:
    break;

  case SYS_WRITE:
    if (f->R.rdi == 1) {
      putbuf (f->R.rsi, f->R.rdx);
    }
    break;

  case SYS_SEEK:
    break;

  case SYS_TELL:
    break;
  case SYS_CLOSE:
    break;

  default:
    break;
  }

  // thread_exit ();
}

static void
sys_halt (void) {
  power_off ();
};

static void
sys_exit (int status) {
  struct list_elem *cur;
  struct thread *t = thread_current ();

  for (cur = list_begin (&t->parent->child_processes);
       cur != list_end (&t->parent->child_processes); cur = list_next (cur)) {

    struct child *c = list_entry (cur, struct child, elem);
    // parent의 자식프로세스 리스트에서 자신을 찾는다
    if (c->child_thread_p->tid == t->tid) {
      c->exit_err = status;
    }
  }

  t->exit_err = status;

  if (t->parent->process_waiting_for == t->tid) {
    sema_up (&t->parent->child_lock);
  }

  thread_exit ();
}
static tid_t
sys_fork (const char *thread_name, struct intr_frame *f) {
  return process_fork (thread_name, f);
};

static int
sys_exec (const char *file) {
  int file_len = strlen (file) + 1;

  char *fn_copy = palloc_get_page (PAL_ZERO);

  if (fn_copy == NULL) {
    sys_exit (-1);
  }

  strlcpy (fn_copy, file, file_len);

  if (process_exec (fn_copy) == -1) {
    return -1;
  }

  // return 0;
  NOT_REACHED ();
};
static int
sys_wait (tid_t tid) {
  printf ("############################\n");
  printf ("####   BEFORE WAIT    ######\n");
  printf ("############################\n");
  return process_wait (tid);
};
// static bool sys_create (const char *file, unsigned initial_size);
// static bool sys_remove (const char *file);
// static int sys_open (const char *file);
// static int sys_filesize (int fd);
// static int sys_read (int fd, void *buffer, unsigned length);
// static int sys_write (int fd, const void *buffer, unsigned length);
// static void sys_seek (int fd, unsigned position);
// static unsigned sys_tell (int fd);
// static void sys_close (int fd);